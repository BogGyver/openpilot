#!/usr/bin/env python3
from cereal import car
from selfdrive.car.tesla.values import CAR
from selfdrive.car import STD_CARGO_KG, gen_empty_fingerprint, scale_rot_inertia, scale_tire_stiffness
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.modules.CFG_module import load_bool_param


class CarInterface(CarInterfaceBase):
  @staticmethod
  def compute_gb(accel, speed):
    # TODO: is this correct?
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "tesla"      
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.enableCamera = True
    ret.openpilotLongitudinalControl = False
    ret.communityFeature = True

    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.5

    #safetyParam
    # BIT - MEANING
    # 0   - Has AP?
    # 1   - Has ACC?
    # 2   - OP Long Control?
    # 3   - HUD Integration?
    # 4   - Body Controls?
    # 5   - Need Radar Emulation

    if candidate == CAR.AP2_MODELS:
      ret.mass = 2100. + STD_CARGO_KG
      ret.wheelbase = 2.959
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 13.5
      ret.safetyParam = 1 + 2 # has AP, ACC
      ret.openpilotLongitudinalControl = False
      ret.safetyModel = car.CarParams.SafetyModel.tesla
    elif candidate == CAR.AP1_MODELS:
      ret.mass = 2100. + STD_CARGO_KG
      ret.wheelbase = 2.959
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 13.5
      ret.safetyParam = 1 + 2 # has AP, ACC
      ret.openpilotLongitudinalControl = False
      ret.safetyModel = car.CarParams.SafetyModel.tesla
    elif candidate == CAR.AP1_MODELX:
      #TODO: update values
      ret.mass = 2560. + STD_CARGO_KG
      ret.wheelbase = 2.964
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 13.5
      ret.safetyParam = 1 + 2 # has AP, ACC
      ret.openpilotLongitudinalControl = False
      ret.safetyModel = car.CarParams.SafetyModel.tesla
    elif candidate == CAR.PREAP_MODELS:
      ret.mass = 2100. + STD_CARGO_KG
      ret.wheelbase = 2.959
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 13.5
      ret.safetyParam = 0 # no AP, ACC
      ret.openpilotLongitudinalControl = False
      ret.safetyModel = car.CarParams.SafetyModel.teslaPreap
      ret.communityFeature = True
    else:
      raise ValueError(f"Unsupported car: {candidate}")

    # set safetyParam flag for OP Long Control
    if ret.openpilotLongitudinalControl:
      ret.safetyParam = ret.safetyParam + 4
    # enable IC integration
    ret.safetyParam = ret.safetyParam + 8
    # enable body controls
    ret.safetyParam = ret.safetyParam + 16
    # enabled radar emulation from carconfig
    if candidate == CAR.PREAP_MODELS and load_bool_param("TinklaUseTeslaRadar",False):
      ret.safetyParam = ret.safetyParam + 32
    # enabled HAO from carconfig
    if load_bool_param("TinklaHao",False):
      ret.safetyParam = ret.safetyParam + 64
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)
    ret.radarOffCan = True
    ret.radarTimeStep = 0.05
    if load_bool_param("TinklaUseTeslaRadar",False):
      ret.radarOffCan = False
    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    events = self.create_common_events(ret)
    if self.CS.autopilot_enabled:
      events.add(car.CarEvent.EventName.invalidLkasSetting)

    ret.events = events.to_msg()
    self.CS.out = ret.as_reader()
    self.CS.DAS_canErrors = 0 if ret.canValid else 1
    self.post_update(c)
    return self.CS.out

  def apply(self, c):
    self.pre_apply(c)
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, 
                          c.actuators, c.cruiseControl.cancel, c.cruiseControl.speedOverride,c.cruiseControl.override,
                          c.hudControl.visualAlert, c.hudControl.audibleAlert, c.hudControl.leftLaneVisible,
                          c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
                          c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)
    self.frame += 1
    return can_sends
