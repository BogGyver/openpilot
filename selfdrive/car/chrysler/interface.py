#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.car.chrysler.values import Ecu, ECU_FINGERPRINT, CAR, FINGERPRINTS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

class CarInterface(CarInterfaceBase):

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)
    ret.carName = "chrysler"
    ret.safetyModel = car.CarParams.SafetyModel.chrysler

    # Speed conversion:              20, 45 mph
    ret.wheelbase = 3.089  # in meters for Pacifica Hybrid 2017
    ret.steerRatio = 16.2 # Pacifica Hybrid 2017
    ret.mass = 2858. + STD_CARGO_KG  # kg curb weight Pacifica Hybrid 2017
    ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15,0.30], [0.03,0.05]]
    ret.lateralTuning.pid.kf = 0.00006   # full torque for 10 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.7
    ret.steerLimitTimer = 0.4

    if candidate in (CAR.JEEP_CHEROKEE, CAR.JEEP_CHEROKEE_2019):
      ret.wheelbase = 2.91  # in meters
      ret.steerRatio = 12.7
      ret.steerActuatorDelay = 0.2  # in seconds

    ret.centerToFront = ret.wheelbase * 0.44

    ret.minSteerSpeed = 3.8  # m/s
    if candidate in (CAR.PACIFICA_2019_HYBRID, CAR.JEEP_CHEROKEE_2019):
      # TODO allow 2019 cars to steer down to 13 m/s if already engaged.
      ret.minSteerSpeed = 17.5  # m/s 17 on the way up, 13 on the way down once engaged.

    # starting with reasonable value for civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)

    ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera) or has_relay
    print("ECU Camera Simulated: {0}".format(ret.enableCamera))

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)

    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    # speeds
    ret.yawRate = self.VM.yaw_rate(ret.steeringAngle * CV.DEG_TO_RAD, ret.vEgo)
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    ret.buttonEvents = []

    # events
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low], gas_resume_speed = 2.)

    if ret.cruiseState.enabled and not self.cruise_enabled_prev:
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    elif not ret.cruiseState.enabled:
      events.append(create_event('pcmDisable', [ET.USER_DISABLE]))

    if ret.vEgo < self.CP.minSteerSpeed:
      events.append(create_event('belowSteerSpeed', [ET.WARNING]))

    ret.events = events

    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed
    self.cruise_enabled_prev = ret.cruiseState.enabled

    # copy back carState packet to CS
    self.CS.out = ret.as_reader()

    return self.CS.out

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    if (self.CS.frame == -1):
      return [] # if we haven't seen a frame 220, then do not update.

    can_sends = self.CC.update(c.enabled, self.CS, c.actuators, c.cruiseControl.cancel, c.hudControl.visualAlert)

    return can_sends
