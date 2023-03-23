#!/usr/bin/env python3
from cereal import car
from selfdrive.car.tesla.values import CAR, CruiseButtons, CAN_AP_POWERTRAIN
from selfdrive.car import STD_CARGO_KG, gen_empty_fingerprint, scale_rot_inertia, scale_tire_stiffness, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.modules.CFG_module import load_bool_param, load_float_param
from panda import Panda
from selfdrive.car.tesla.tunes import LongTunes, set_long_tune, ACCEL_LOOKUP_BP, ACCEL_MAX_LOOKUP_V, ACCEL_MIN_LOOKUP_V, ACCEL_REG_LOOKUP_V, ACCEL_AP_MAX_LOOKUP_V
from common.numpy_fast import interp
from selfdrive.config import Conversions as CV

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
HAS_IBOOSTER_ECU = load_bool_param("TinklaHasIBooster",False)
ACCEL_PROFILE = int(load_float_param("TinklaAccelProfile", 2.0)-1)

UNSAFE_DISABLE_DISENGAGE_ON_GAS = 1
UNSAFE_DISABLE_STOCK_AEB = 2
UNSAFE_RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX = 8

#this function is called from longitudinal_planner only for limits
def get_tesla_accel_limits(CP, current_speed):
  a_min = 0.
  a_max = interp(current_speed,ACCEL_LOOKUP_BP,ACCEL_MAX_LOOKUP_V[ACCEL_PROFILE])
  if CP.carFingerprint == CAR.PREAP_MODELS and not HAS_IBOOSTER_ECU:
    a_min = interp(current_speed,ACCEL_LOOKUP_BP,ACCEL_REG_LOOKUP_V)
  elif CP.carFingerprint == CAR.PREAP_MODELS and HAS_IBOOSTER_ECU:
    a_min = interp(current_speed,ACCEL_LOOKUP_BP,ACCEL_MIN_LOOKUP_V)
  else:
    a_max = interp(current_speed,ACCEL_LOOKUP_BP,ACCEL_AP_MAX_LOOKUP_V[ACCEL_PROFILE])
    a_min = interp(current_speed,ACCEL_LOOKUP_BP,ACCEL_MIN_LOOKUP_V)
  return a_min, a_max

class CarInterface(CarInterfaceBase):
  @staticmethod
  def compute_gb(accel, speed):
    # TODO: is this correct?
    return float(accel) 

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return get_tesla_accel_limits(CP,current_speed)
    

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    if load_bool_param("TinklaForceTeslaPreAP",False):
      candidate = CAR.PREAP_MODELS
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "tesla"      
    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.longitudinalActuatorDelayUpperBound = 0.5 # s
    ret.radarTimeStep = (1.0 / 8) # 8Hz

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    ret.steerRateCost = 0.5

    #safetyParam
    # BIT - MEANING
    #  Panda.FLAG_TESLA_POWERTRAIN = 1
    #  Panda.FLAG_TESLA_LONG_CONTROL = 2
    #  Panda.FLAG_TESLA_HAS_IC_INTEGRATION = 8
    #  Panda.FLAG_TESLA_HAS_AP = 16
    #  Panda.FLAG_TESLA_NEED_RADAR_EMULATION = 32
    #  Panda.FLAG_TESLA_HAO = 64
    #  Panda.FLAG_TESLA_IBOOSTER = 128

    safetyParam = 0
    ret.wheelSpeedFactor = 1.
    if candidate in (CAR.AP2_MODELS, CAR.AP1_MODELS):
      ret.mass = 2100. + STD_CARGO_KG
      ret.wheelbase = 2.959
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 15
      safetyParam = safetyParam | Panda.FLAG_TESLA_HAS_AP # has AP, ACC
      ret.openpilotLongitudinalControl = False
      set_long_tune(ret.longitudinalTuning, LongTunes.AP)
    elif candidate == CAR.AP1_MODELX:
      #TODO: update values
      ret.mass = 2560. + STD_CARGO_KG
      ret.wheelbase = 2.964
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 15
      safetyParam = safetyParam | Panda.FLAG_TESLA_HAS_AP  # has AP, ACC
      ret.openpilotLongitudinalControl = False
      set_long_tune(ret.longitudinalTuning, LongTunes.AP)
    elif candidate == CAR.PREAP_MODELS:
      ret.mass = 2100. + STD_CARGO_KG
      ret.wheelbase = 2.959
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 15
      ret.openpilotLongitudinalControl = False
      if HAS_IBOOSTER_ECU:
        set_long_tune(ret.longitudinalTuning, LongTunes.IBST)
      elif load_bool_param("TinklaEnablePedal",False):
        set_long_tune(ret.longitudinalTuning, LongTunes.PEDAL)
      else:
        set_long_tune(ret.longitudinalTuning, LongTunes.ACC)
    else:
      raise ValueError(f"Unsupported car: {candidate}")
    ret.minEnableSpeed = -1. 
    # enable IC integration - always enabled
    # enable body controls - always enabled
    # enabled radar emulation from carconfig
    if candidate == CAR.PREAP_MODELS and load_bool_param("TinklaUseTeslaRadar",False):
      safetyParam = safetyParam | Panda.FLAG_TESLA_NEED_RADAR_EMULATION
      if load_bool_param("TinklaTeslaRadarBehindNosecone",False):
        safetyParam = safetyParam | Panda.FLAG_TESLA_RADAR_BEHIND_NOSECONE
    # enabled HAO from carconfig
    if load_bool_param("TinklaHao",False):
      safetyParam = safetyParam | Panda.FLAG_TESLA_ENABLE_HAO
    if load_bool_param("TinklaEnableOPLong",False) or ret.openpilotLongitudinalControl:
      safetyParam = safetyParam | Panda.FLAG_TESLA_LONG_CONTROL
      ret.openpilotLongitudinalControl = True
    if load_bool_param("TinklaHasIcIntegration",False):
      safetyParam = safetyParam | Panda.FLAG_TESLA_HAS_IC_INTEGRATION
    # true if car has ibooster
    if (candidate == CAR.PREAP_MODELS and load_bool_param("TinklaHasIBooster",False)) or (candidate != CAR.PREAP_MODELS):
      safetyParam = safetyParam | Panda.FLAG_TESLA_HAS_IBOOSTER
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)
    ret.radarOffCan = False
    if candidate == CAR.PREAP_MODELS:
      ret.radarOffCan = not load_bool_param("TinklaUseTeslaRadar",False)
    # set safetyParam flag for OP Long Control
    if ret.openpilotLongitudinalControl:
      safetyParam = safetyParam | Panda.FLAG_TESLA_LONG_CONTROL

    if candidate == CAR.AP2_MODELS:
      # Check if we have messages on an auxiliary panda, and that 0x2bf (DAS_control) is present on the AP powertrain bus
      # If so, we assume that it is connected to the longitudinal harness.
      if (CAN_AP_POWERTRAIN[candidate] in fingerprint.keys()) and (0x2bf in fingerprint[CAN_AP_POWERTRAIN[candidate]].keys()):
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs = [
          get_safety_config(car.CarParams.SafetyModel.tesla, safetyParam | Panda.FLAG_TESLA_LONG_CONTROL),
          get_safety_config(car.CarParams.SafetyModel.tesla, safetyParam | Panda.FLAG_TESLA_LONG_CONTROL | Panda.FLAG_TESLA_POWERTRAIN),
        ]
      else:
        ret.openpilotLongitudinalControl = False
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, safetyParam)]
    else:
        ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, safetyParam)]
    if candidate == CAR.PREAP_MODELS:
      ret.stoppingDecelRate = 1. 
      ret.stoppingControl = True
      ret.stopAccel = -2.0 
      #ret.vEgoStopping = 5 * CV.MPH_TO_MS
    else:
      ret.stoppingDecelRate = 0.6 #since we don't use the PID, this means a jerk in acceleration by x m/s^3
      ret.stoppingControl = True
      ret.stopAccel = -2.0
    ret.unsafeMode = UNSAFE_DISABLE_DISENGAGE_ON_GAS | UNSAFE_RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX
    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)
    if self.CP.carFingerprint == CAR.PREAP_MODELS:
      ret.canValid = self.cp.can_valid
      ret.canErrorId = self.cp.error_address
    else:
      ret.canValid = self.cp.can_valid and self.cp_cam.can_valid
      ret.canErrorId = max(self.cp.error_address, self.cp_cam.error_address)

    self.post_update(c,ret)

    buttonEvents = []

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != 0:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == CruiseButtons.DECEL_SET:
        be.type = ButtonType.decelCruise
      elif but == CruiseButtons.CANCEL:
        be.type = ButtonType.cancel
      elif but == CruiseButtons.MAIN:
        be.type = ButtonType.setCruise
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents
    
    events = self.create_common_events(ret)
    if self.CS.autopilot_enabled:
      events.add(car.CarEvent.EventName.invalidLkasSetting)
    if ret.gasPressed and self.CS.adaptive_cruise_enabled:
      events.add(EventName.gasPressed)

    if self.CS.longCtrlEvent:
      events.add(self.CS.longCtrlEvent)
      self.CS.longCtrlEvent = None

    if self.CS.pccEvent:
      events.add(self.CS.pccEvent)

    ret.events = events.to_msg()
    
    self.CS.out = ret.as_reader()
    self.CS.DAS_canErrors = 0 if ret.canValid else 1
    
    return self.CS.out

  def apply(self, c):
    self.pre_apply(c)
    ret = self.CC.update(c, c.enabled, self.CS, self.frame, c.actuators, 
        c.cruiseControl.cancel,c.cruiseControl.speedOverride,c.cruiseControl.override,
                          c.hudControl.visualAlert, c.hudControl.audibleAlert, c.hudControl.leftLaneVisible,
                          c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
                          c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)
    self.frame += 1
    return ret
