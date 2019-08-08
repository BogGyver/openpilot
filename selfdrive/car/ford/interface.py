#!/usr/bin/env python
from common.realtime import sec_since_boot
from cereal import car
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.ford.carstate import CarState, get_can_parser
from selfdrive.car.ford.values import MAX_ANGLE
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness


class CarInterface(object):
  def __init__(self, CP, CarController):
    self.CP = CP
    self.VM = VehicleModel(CP)

    self.frame = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.cruise_enabled_prev = False

    # *** init the major players ***
    self.CS = CarState(CP)

    self.cp = get_can_parser(CP)

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name, CP.enableCamera, self.VM)

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.0

  @staticmethod
  def get_params(candidate, fingerprint, vin="", is_panda_black=False):

    ret = car.CarParams.new_message()

    ret.carName = "ford"
    ret.carFingerprint = candidate
    ret.carVin = vin
    ret.isPandaBlack = is_panda_black

    ret.safetyModel = car.CarParams.SafetyModel.ford

    # pedal
    ret.enableCruise = True

    ret.wheelbase = 2.85
    ret.steerRatio = 14.8
    ret.mass = 3045. * CV.LB_TO_KG + STD_CARGO_KG
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.01], [0.005]]     # TODO: tune this
    ret.lateralTuning.pid.kf = 1. / MAX_ANGLE   # MAX Steer angle to normalize FF
    ret.steerActuatorDelay = 0.1  # Default delay, not measured yet
    ret.steerRateCost = 1.0
    ret.centerToFront = ret.wheelbase * 0.44
    tire_stiffness_factor = 0.5328

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    ret.minEnableSpeed = -1.

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.
    ret.steerControlType = car.CarParams.SteerControlType.angle

    # steer, gas, brake limitations VS speed
    ret.steerMaxBP = [0.]  # breakpoints at 1 and 40 kph
    ret.steerMaxV = [1.0]  # 2/3rd torque allowed above 45 kph
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [0.5]
    ret.brakeMaxBP = [5., 20.]
    ret.brakeMaxV = [1., 0.8]

    ret.enableCamera = not any(x for x in [970, 973, 984] if x in fingerprint) or is_panda_black
    ret.openpilotLongitudinalControl = False
    cloudlog.warn("ECU Camera Simulated: %r", ret.enableCamera)

    ret.steerLimitAlert = False
    ret.stoppingControl = False
    ret.startAccel = 0.0

    ret.longitudinalTuning.deadzoneBP = [0., 9.]
    ret.longitudinalTuning.deadzoneV = [0., .15]
    ret.longitudinalTuning.kpBP = [0., 5., 35.]
    ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
    ret.longitudinalTuning.kiBP = [0., 35.]
    ret.longitudinalTuning.kiV = [0.54, 0.36]

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(int(sec_since_boot() * 1e9), can_strings)

    self.CS.update(self.cp)

    # create message
    ret = car.CarState.new_message()

    ret.canValid = self.cp.can_valid

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringPressed = self.CS.steer_override

    # gas pedal
    ret.gas = self.CS.user_gas / 100.
    ret.gasPressed = self.CS.user_gas > 0.0001
    ret.brakePressed = self.CS.brake_pressed
    ret.brakeLights = self.CS.brake_lights

    ret.cruiseState.enabled = not (self.CS.pcm_acc_status in [0, 3])
    ret.cruiseState.speed = self.CS.v_cruise_pcm
    ret.cruiseState.available = self.CS.pcm_acc_status != 0

    ret.genericToggle = self.CS.generic_toggle

    # events
    events = []

    if self.CS.steer_error:
      events.append(create_event('steerUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))

    # enable request in prius is simple, as we activate when Toyota is active (rising edge)
    if ret.cruiseState.enabled and not self.cruise_enabled_prev:
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    elif not ret.cruiseState.enabled:
      events.append(create_event('pcmDisable', [ET.USER_DISABLE]))

    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    if (ret.gasPressed and not self.gas_pressed_prev) or \
       (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    if ret.gasPressed:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))

    if self.CS.lkas_state not in [2, 3] and ret.vEgo > 13.* CV.MPH_TO_MS and ret.cruiseState.enabled:
      events.append(create_event('steerTempUnavailableMute', [ET.WARNING]))

    ret.events = events

    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed
    self.cruise_enabled_prev = ret.cruiseState.enabled

    return ret.as_reader()

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                               c.hudControl.visualAlert, c.cruiseControl.cancel)

    self.frame += 1
    return can_sends
