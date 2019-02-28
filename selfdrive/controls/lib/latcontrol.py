from selfdrive.controls.lib.pid import PIController
from common.numpy_fast import interp
from common.realtime import sec_since_boot
from cereal import car
import math
import numpy as np

_DT = 0.01    # 100Hz
_DT_MPC = 0.05  # 20Hz


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)


def apply_deadzone(angle, deadzone):
  if angle > deadzone:
    angle -= deadzone
  elif angle < -deadzone:
    angle += deadzone
  else:
    angle = 0.
  return angle


class LatControl(object):
  def __init__(self, CP):

    if CP.steerResistance > 0 and CP.steerReactance >= 0 and CP.steerInductance > 0:
      self.smooth_factor = CP.steerInductance * 2.0 * CP.steerActuatorDelay / _DT    # Multiplier for inductive component (feed forward)
      self.projection_factor = CP.steerReactance * CP.steerActuatorDelay / 2.0       # Mutiplier for reactive component (PI)
      self.accel_limit = 2.0 / CP.steerResistance                                    # Desired acceleration limit to prevent "whip steer" (resistive component)
      self.ff_angle_factor = 1.0                                                     # Kf multiplier for angle-based feed forward
      self.ff_rate_factor = 10.0                                                      # Kf multiplier for rate-based feed forward
      # Eliminate break-points, since they aren't needed (and would cause problems for resonance)
      KpV = [np.interp(25.0, CP.steerKpBP, CP.steerKpV)]
      KiV = [np.interp(25.0, CP.steerKiBP, CP.steerKiV) * _DT / self.projection_factor]
      self.pid = PIController(([0.], KpV),
                              ([0.], KiV),
                              k_f=CP.steerKf, pos_limit=1.0)
    else:
      self.pid = PIController((CP.steerKpBP, CP.steerKpV),
                              (CP.steerKiBP, CP.steerKiV),
                              k_f=CP.steerKf, pos_limit=1.0)
      self.smooth_factor = 1.0
      self.projection_factor = 0.0
      self.accel_limit = 0.0
      self.ff_angle_factor = 1.0
      self.ff_rate_factor = 0.0
    self.last_cloudlog_t = 0.0
    self.prev_angle_rate = 0
    self.feed_forward = 0.0
    self.last_mpc_ts = 0.0
    self.angle_steers_des_time = 0.0
    self.angle_steers_des_mpc = 0.0
    self.steer_counter = 1.0
    self.steer_counter_prev = 0.0
    self.rough_steers_rate = 0.0
    self.prev_angle_steers = 0.0
    self.calculate_rate = True

  def reset(self):
    self.pid.reset()

  def update(self, active, v_ego, angle_steers, angle_rate, angle_offset, steer_override, CP, VM, path_plan):

    if angle_rate == 0.0 and self.calculate_rate:
      if angle_steers != self.prev_angle_steers:
        self.steer_counter_prev = self.steer_counter
        self.rough_steers_rate = (self.rough_steers_rate + 100.0 * (angle_steers - self.prev_angle_steers) / self.steer_counter_prev) / 2.0
        self.steer_counter = 0.0
      elif self.steer_counter >= self.steer_counter_prev:
        self.rough_steers_rate = (self.steer_counter * self.rough_steers_rate) / (self.steer_counter + 1.0)
      self.steer_counter += 1.0
      angle_rate = self.rough_steers_rate

      # Don't use accelerated rate unless it's from CAN
      accelerated_angle_rate = angle_rate
    else:
      # If non-zero angle_rate is provided, use it instead
      self.calculate_rate = False
      # Use steering rate from the last 2 samples to estimate acceleration for a likely future steering rate
      accelerated_angle_rate = 2.0 * angle_rate - self.prev_angle_rate


    cur_time = sec_since_boot()
    self.angle_steers_des_time = cur_time

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      self.feed_forward = 0.0
      self.pid.reset()
      self.angle_steers_des = angle_steers
    else:
      # Interpolate desired angle between MPC updates
      self.angle_steers_des = np.interp(cur_time, path_plan.mpcTimes, path_plan.mpcAngles)
      self.angle_steers_des_time = cur_time

      # Determine the target steer rate for desired angle, but prevent the acceleration limit from being exceeded
      # Restricting the steer rate creates the resistive component needed for resonance
      restricted_steer_rate = np.clip(self.angle_steers_des - float(angle_steers) , float(accelerated_angle_rate) - self.accel_limit,
                                                                                    float(accelerated_angle_rate) + self.accel_limit)

      # Determine projected desired angle that is within the acceleration limit (prevent the steering wheel from jerking)
      projected_angle_steers_des = self.angle_steers_des + self.projection_factor * restricted_steer_rate

      # Determine future angle steers using accellerated steer rate
      projected_angle_steers = float(angle_steers) + self.projection_factor * float(accelerated_angle_rate)

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      deadzone = 0.0

      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # Decide which feed forward mode should be used (angle or rate).  Use more dominant mode, but only if conditions are met
        # Spread feed forward out over a period of time to make it inductive (for resonance)
        if abs(self.ff_rate_factor * float(restricted_steer_rate)) > abs(self.ff_angle_factor * float(self.angle_steers_des) - float(angle_offset)) - 0.5 \
            and (abs(float(restricted_steer_rate)) > abs(accelerated_angle_rate) or (float(restricted_steer_rate) < 0) != (accelerated_angle_rate < 0)) \
            and (float(restricted_steer_rate) < 0) == (float(self.angle_steers_des) - float(angle_offset) - 0.5 < 0):
          self.feed_forward = (((self.smooth_factor - 1.) * self.feed_forward) + self.ff_rate_factor * v_ego**2 * float(restricted_steer_rate)) / self.smooth_factor
        elif abs(self.angle_steers_des - float(angle_offset)) > 0.5:
          self.feed_forward = (((self.smooth_factor - 1.) * self.feed_forward) + self.ff_angle_factor * v_ego**2 \
                              * float(apply_deadzone(float(self.angle_steers_des) - float(angle_offset), 0.5))) / self.smooth_factor
        else:
          self.feed_forward = (((self.smooth_factor - 1.) * self.feed_forward) + 0.0) / self.smooth_factor

        # Use projected desired and actual angles instead of "current" values, in order to make PI more reactive (for resonance)
        output_steer = self.pid.update(projected_angle_steers_des, projected_angle_steers, check_saturation=(v_ego > 10),
                                        override=steer_override, feedforward=self.feed_forward, speed=v_ego, deadzone=deadzone)

    self.sat_flag = self.pid.saturated
    self.prev_angle_rate = angle_rate
    self.prev_angle_steers = angle_steers

    # return MPC angle in the unused output (for ALCA)
    if CP.steerControlType == car.CarParams.SteerControlType.torque:
      return output_steer, self.angle_steers_des
    else:
      return self.angle_steers_des_mpc, float(self.angle_steers_des)
