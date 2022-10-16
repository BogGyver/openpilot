from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.pid_real import  PIDController
from selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.modeld.constants import T_IDXS
import json
from selfdrive.car.tesla.values import USE_REAL_PID, kdBp, kdV, V_PID_FILE,gasMaxBP, gasMaxV, brakeMaxBP, brakeMaxV

LongCtrlState = car.CarControl.Actuators.LongControlState

# As per ISO 15622:2018 for all speeds
ACCEL_MIN_ISO = -3.5  # m/s^2
ACCEL_MAX_ISO = 2.0  # m/s^2


def long_control_state_trans(CP, active, long_control_state, v_ego, v_target_future,
                             brake_pressed, cruise_standstill):
  """Update longitudinal control state machine"""
  stopping_condition = (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < CP.vEgoStopping and
                        (v_target_future < CP.vEgoStopping or brake_pressed))

  starting_condition = v_target_future > CP.vEgoStarting and not cruise_standstill

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl():
  def __init__(self, CP):
    self.long_control_state = LongCtrlState.off  # initialized to off
    
    self.USE_REAL_PID = USE_REAL_PID[CP.carFingerprint]
    if self.USE_REAL_PID:
      self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
          (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
          (kdBp,kdV),
          rate=1 / DT_CTRL,
          sat_limit=0.8,
          convert=None)
      self.load_pid()
    else:
      self.pid = PIController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
          (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
          rate=1 / DT_CTRL)
    
    self.v_pid = 0.0
    self.last_output_accel = 0.0

  def load_pid(self):
    try:
      v_pid_json = open(V_PID_FILE)
      data = json.load(v_pid_json)
      if self.pid:
        self.pid.p = data["p"]
        self.pid.i = data["i"]
        if "d" not in data:
          self.pid.d = 0.01
        else:
          self.pid.d = data["d"]
        self.pid.f = data["f"]
      else:
        print("self.pid not initialized!")
    except IOError:
      print("file not present, creating at next reset")

  def save_pid(self):
    data = {}
    data["p"] = self.pid.p
    data["i"] = self.pid.i
    data["d"] = self.pid.d
    data["f"] = self.pid.f
    try:
      with open(V_PID_FILE, "w") as outfile:
        json.dump(data, outfile)
    except IOError:
      print("PDD pid parameters could not be saved to file")

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    if not self.USE_REAL_PID:
      self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, CP, long_plan, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    # TODO estimate car specific lag, use .15s for now
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_lower = interp(CP.longitudinalActuatorDelayLowerBound, T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - speeds[0])/CP.longitudinalActuatorDelayLowerBound - long_plan.accels[0]

      v_target_upper = interp(CP.longitudinalActuatorDelayUpperBound, T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - speeds[0])/CP.longitudinalActuatorDelayUpperBound - long_plan.accels[0]
      a_target = min(a_target_lower, a_target_upper)

      v_target = speeds[0]
      v_target_future = speeds[-1]
    else:
      v_target = 0.0
      v_target_future = 0.0
      a_target = 0.0

    # TODO: This check is not complete and needs to be enforced by MPC
    #a_target = clip(a_target, ACCEL_MIN_ISO, ACCEL_MAX_ISO)
    #ISO CRAP 2m/s^2 = 20m/s in 10s.... 0-40mph in 10s...
    # even 3.5m/s^2 is 0-60 in 8s

    a_target = clip(a_target, accel_limits[0], accel_limits[1])

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # Actuation limits
    if self.USE_REAL_PID:
      gas_max = interp(CS.vEgo, gasMaxBP, gasMaxV)
      brake_max = interp(CS.vEgo, brakeMaxBP, brakeMaxV)
      self.pid.neg_limit = -brake_max
      self.pid.pos_limit = gas_max

    # Update state machine
    output_accel = self.last_output_accel
    prev_long_control_state = self.long_control_state
    self.long_control_state = long_control_state_trans(CP, active, self.long_control_state, CS.vEgo,
                                                       v_target_future, CS.brakePressed,
                                                       CS.cruiseState.standstill)
                                                      
    if (self.long_control_state == LongCtrlState.off or self.long_control_state == LongCtrlState.stopping) and prev_long_control_state == LongCtrlState.pid:
      #save pid state on disengage
      if self.USE_REAL_PID:
        self.save_pid()

    if self.long_control_state == LongCtrlState.off or CS.gasPressed:
      self.reset(CS.vEgo)
      output_accel = 0.

    # tracking objects and driving
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not CP.stoppingControl and CS.vEgo < 1.5 and v_target_future < 0.7 and v_target_future < self.v_pid
      deadzone = interp(CS.vEgo, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      output_accel = self.pid.update(self.v_pid, CS.vEgo, speed=CS.vEgo, deadzone=deadzone, feedforward=a_target, freeze_integrator=freeze_integrator)

      if prevent_overshoot:
        output_accel = min(output_accel, 0.0)

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      if not CS.standstill or output_accel > CP.stopAccel:
        output_accel -= CP.stoppingDecelRate * DT_CTRL
      if self.USE_REAL_PID:
        output_accel = clip(output_accel, -brake_max, gas_max)
      else:
        output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
      self.reset(CS.vEgo)

    self.last_output_accel = output_accel
    
    final_accel = clip(output_accel, accel_limits[0], accel_limits[1])
    if self.USE_REAL_PID:
      final_accel = clip(output_accel, -brake_max, gas_max)

    return final_accel
