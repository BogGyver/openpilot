import copy
from cereal import car
from selfdrive.car.tesla.values import DBC, GEAR_MAP, DOORS, BUTTONS, CAR, CruiseButtons, CruiseState
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.modules.CFG_module import load_bool_param

#tesla uses various tires, this is for now for the 245/45R19s or 245/35ZR21s and they are 27.8" diameter = 0.353m
#TODOBB: sometimes tesla on the rear has 265/35ZR21 and they are 28.3" diameter 0.359m
#for now OP does not allow for various uom per tire ... to look into it
WHEEL_RADIUS = 0.353

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    self.button_states = {button.event_type: False for button in BUTTONS}
    self.can_define = CANDefine(DBC[CP.carFingerprint]['chassis'])

    self.hands_on_level = 0
    self.steer_warning = None
    self.acc_state = 0

    # Needed by carcontroller
    self.msg_stw_actn_req = None

    self.das_steeringControl_counter = -1
    self.das_status_counter = -1
    self.autopilot_enabled = False

    #to control hazard lighgts
    self.needs_hazard = False

    #path info
    self.laneWidth = 0
    self.lProb = 0.
    self.rProb = 0.
    self.curvC0 = 0.
    self.curvC1 = 0.
    self.curvC2 = 0.
    self.curvC3 = 0.
    self.laneToLeft = 0
    self.laneToRight = 0
    self.lLine = 0
    self.rLine = 0

    self.acc_speed_kph = 0.

    #IC integration
    self.DAS_gas_to_resume = 0
    self.DAS_025_steeringOverride = 0 #use for manual steer?
    self.DAS_202_noisyEnvironment = 0 #use for planner error?
    self.DAS_206_apUnavailable = 0 #Ap disabled from CID
    self.DAS_207_lkasUnavailable = 0 #use for manual not in drive?
    self.DAS_208_rackDetected = 0 #use for low battery?
    self.DAS_211_accNoSeatBelt = 0
    self.DAS_216_driverOverriding = 0
    self.DAS_219_lcTempUnavailableSpeed = 0
    self.DAS_220_lcTempUnavailableRoad = 0
    self.DAS_221_lcAborting = 0
    self.DAS_222_accCameraBlind = 0 #we will see what we can use this for
    self.stopSignWarning = 0
    self.stopLightWarning = 0
    self.DAS_canErrors = 0
    self.DAS_notInDrive = 0
    self.DAS_fusedSpeedLimit = 0
    self.baseMapSpeedLimitMPS = 0
    self.fleet_speed_state = 0

    self.cruiseEnabled = False
    self.cruiseDelay = False
    self.carNotInDrive = True

    self.speed_units = "MPH"
    self.tap_direction = 0

    self.op_lkas_enabled = False

    #preAP long
    self.speed_control_enabled = 0
    self.cc_state = 1
    self.adaptive_cruise = 0
    self.apFollowTimeInS = 2.5  # time in seconds to follow
    self.pcc_available = False
    self.torqueLevel = 0.0
    self.cruise_state = None
    self.enableHumanLongControl = load_bool_param("TinklaForceTeslaPreAP", False)
    self.enableICIntegration = load_bool_param("TinklaHasIcIntegration", False)
    self.pedalcanzero = load_bool_param("TinklaPedalCanZero",False)
    self.has_ibooster_ecu = load_bool_param("TinklaHasIBooster",False)
    self.brakeUnavailable = True
    self.realBrakePressed = False
    self.userSpeedLimitOffsetMS = 0

    #data to spam GTW_ESP1
    self.gtw_esp1 = None
    self.gtw_esp1_id = -1
    self.gtw_esp1_last_sent_id = 0
    self.prev_gtw_esp1_bw_req = 0
    self.gtw_esp1_bw_req = 0

    #pedal interceptor variables
    self.pedal_idx = self.prev_pedal_idx = 0
    self.prev_pedal_interceptor_state = self.pedal_interceptor_state = 0
    self.pedal_interceptor_value = 0.0
    self.pedal_interceptor_value2 = 0.0
    self.teslaModel = "S"

  def _convert_to_DAS_fusedSpeedLimit(self, speed_limit_uom, speed_limit_type):
    if speed_limit_uom > 0:
      if speed_limit_type == 0x1E: # Autobahn with no speed limit
        return 0x1F * 5 # no speed limit sign
      return int(speed_limit_uom + 0.5)
    else:
      if speed_limit_type == 0x1F: # SNA (parking lot, no public road, etc.)
        return 0 # no sign or show 35 for debug
      return 5 # show 5 kph/mph for unknown limit where we should have one

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = (ret.vEgo < 0.1)

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["ESP_B"]["ESP_wheelPulseCountFrL"],
      cp.vl["ESP_B"]["ESP_wheelPulseCountFrR"],
      cp.vl["ESP_B"]["ESP_wheelPulseCountReL"],
      cp.vl["ESP_B"]["ESP_wheelPulseCountReR"],
      unit=WHEEL_RADIUS,
    )

    # Gas pedal
    ret.gas = cp.vl["DI_torque1"]["DI_pedalPos"] / 100.0
    ret.gasPressed = (ret.gas > 0) 
    if self.enableHAO:
      self.DAS_216_driverOverriding = 1 if (ret.gas > 0) else 0
      ret.gas = 0
      ret.gasPressed = False

    # Brake pedal
    ret.brake = 0
    if self.has_ibooster_ecu:
      self.realBrakePressed = bool(cp.vl["ECU_BrakeStatus"]['DriverBrakeApplied'])
      ret.brakeLights = bool(cp.vl["ECU_BrakeStatus"]['BrakeApplied'])
      self.brakeUnavailable = not bool(cp.vl["ECU_BrakeStatus"]['BrakeOK'])
    else:
      self.realBrakePressed = bool(cp.vl["BrakeMessage"]["driverBrakeStatus"] != 1)
    ret.brakePressed = self.realBrakePressed
    # Steering wheel
    
    self.steer_warning = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(cp.vl["EPAS_sysStatus"]["EPAS_eacErrorCode"]), None)
    steer_status = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(cp.vl["EPAS_sysStatus"]["EPAS_eacStatus"]), None)
    ret.steeringAngleDeg = -cp.vl["EPAS_sysStatus"]["EPAS_internalSAS"]
    ret.steeringTorque = -cp.vl["EPAS_sysStatus"]["EPAS_torsionBarTorque"]

    ret.steeringRateDeg = -cp.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"] # This is from a different angle sensor, and at different rate
    self.hands_on_level = cp.vl["EPAS_sysStatus"]["EPAS_handsOnLevel"]
    ret.steeringPressed = (self.hands_on_level > 0)
    ret.steerError = steer_status == "EAC_FAULT"
    ret.steerWarning = self.steer_warning != "EAC_ERROR_IDLE"
    self.torqueLevel = cp.vl["DI_torque1"]["DI_torqueMotor"]

    # Cruise state
    #cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp.vl["DI_state"]["DI_cruiseState"]), None)
    #speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp.vl["DI_state"]["DI_speedUnits"]), None)
    #acc_enabled = (cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL"))
    #ret.cruiseState.enabled = acc_enabled
    #if speed_units == "KPH":
    #  ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS
    #elif speed_units == "MPH":
    #  ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS
    #ret.cruiseState.available = ((cruise_state == "STANDBY") or ret.cruiseState.enabled)
    #et.cruiseState.standstill = False # This needs to be false, since we can resume from stop without sending anything special


    autopilot_status = None
    if (not self.CP.carFingerprint == CAR.PREAP_MODELS):
      autopilot_status = self.can_define.dv["DAS_status"]["DAS_autopilotState"].get(int(cp_cam.vl["DAS_status"]["DAS_autopilotState"]), None)
    autopark_status = None
    eac_status = None
    if (not self.CP.carFingerprint == CAR.PREAP_MODELS):
      eac_status = self.can_define.dv["DAS_pscControl"]["DAS_eacState"].get(int(cp_cam.vl["DAS_pscControl"]["DAS_eacState"]), None)
      autopark_status = self.can_define.dv["DAS_pscControl"]["DAS_pscParkState"].get(int(cp_cam.vl["DAS_pscControl"]["DAS_pscParkState"]), None)
    summon_or_autopark_enabled = (
        eac_status in ["EAC_ACTIVE"] 
        or autopark_status in ["SUMMON", "COMPLETE", "ABORT", "PARALLEL_PULL_OUT_TO_RIGHT", 
        "PARALLEL_PULL_OUT_TO_LEFT", "PARK_RIGHT_CROSS", "PARK_RIGHT_PARALLEL", 
        "PARK_LEFT_CROSS", "PARK_LEFT_PARALLEL"]
    )
    self.cruise_state = cp.vl["DI_state"]["DI_cruiseState"]
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp.vl["DI_state"]["DI_cruiseState"]), None)
    self.speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp.vl["DI_state"]["DI_speedUnits"]), None)
    self.v_cruise_actual = cp.vl["DI_state"]["DI_cruiseSet"]
    if self.speed_units == "MPH":
        self.v_cruise_actual = self.v_cruise_actual * CV.MPH_TO_KPH
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = cp.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"]
    
    
    if (self.CP.carFingerprint != CAR.PREAP_MODELS):
      acc_enabled = (cruise_state in ["ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL"])
      self.autopilot_enabled = (autopilot_status in ["ACTIVE_1", "ACTIVE_2"]) #, "ACTIVE_NAVIGATE_ON_AUTOPILOT"])
      self.cruiseEnabled = acc_enabled and not self.autopilot_enabled and not summon_or_autopark_enabled
      ret.cruiseState.enabled = self.cruiseEnabled and self.cruiseDelay
      if self.speed_units == "KPH":
        ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS
      elif self.speed_units == "MPH":
        ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS
      ret.cruiseState.available = ((cruise_state == "STANDBY") or ret.cruiseState.enabled)
      ret.cruiseState.standstill = False #(cruise_state == "STANDSTILL")
    else:
      ret.cruiseState.speed = self.acc_speed_kph * CV.KPH_TO_MS
      ret.cruiseState.enabled = self.cruiseEnabled and (not ret.doorOpen) and (ret.gearShifter == car.CarState.GearShifter.drive) and (not ret.seatbeltUnlatched)
      ret.cruiseState.available = True
      ret.cruiseState.standstill = False

    #speed limit
    msu = cp.vl['UI_gpsVehicleSpeed']["UI_mapSpeedLimitUnits"]
    map_speed_uom_to_ms = CV.KPH_TO_MS if msu == 1 else CV.MPH_TO_MS
    map_speed_ms_to_uom = CV.MS_TO_KPH if msu == 1 else CV.MS_TO_MPH
    
    speed_limit_type = int(cp.vl["UI_driverAssistMapData"]["UI_mapSpeedLimit"])
    rdSignMsg = cp.vl["UI_driverAssistRoadSign"]["UI_roadSign"]
    if rdSignMsg == 3: # ROAD_SIGN_SPEED_LIMIT
      self.baseMapSpeedLimitMPS = cp.vl["UI_driverAssistRoadSign"]["UI_baseMapSpeedLimitMPS"]
      # we round the speed limit in the map's units of measurement to fix noisy data (there are no signs with a limit of 79.2 kph)
      self.baseMapSpeedLimitMPS = int(self.baseMapSpeedLimitMPS * map_speed_ms_to_uom + 0.99) / map_speed_ms_to_uom
    if self.CP.carFingerprint != CAR.PREAP_MODELS and self.baseMapSpeedLimitMPS > 0 and (speed_limit_type != 0x1F or self.baseMapSpeedLimitMPS >= 5.56):
      self.speed_limit_ms = self.baseMapSpeedLimitMPS # this one is earlier than the actual sign but can also be unreliable, so we ignore it on SNA at higher speeds
    else:
      self.speed_limit_ms = cp.vl['UI_gpsVehicleSpeed']["UI_mppSpeedLimit"] * map_speed_uom_to_ms
    self.DAS_fusedSpeedLimit = self._convert_to_DAS_fusedSpeedLimit(self.speed_limit_ms * map_speed_ms_to_uom, speed_limit_type)
    if self.DAS_fusedSpeedLimit > 1:
      self.fleet_speed_state = 2
    else:
      self.fleet_speed_state = 0
    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_torque2"]["DI_gear"].get(int(cp.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]
    self.DAS_notInDrive = 0 if ret.gearShifter == car.CarState.GearShifter.drive else 1
    self.carNotInDrive = (not (ret.gearShifter == car.CarState.GearShifter.drive))

    # Buttons
    buttonEvents = []
    for button in BUTTONS:
      state = (cp.vl[button.can_addr][button.can_msg] in button.values)
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        buttonEvents.append(event)
      self.button_states[button.event_type] = state 
    ret.buttonEvents = buttonEvents

    # Doors
    ret.doorOpen = any([(self.can_define.dv["GTW_carState"][door].get(int(cp.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS])

    # Blinkers are used for Comma ALCA
    # we use tap to trigger Comma ALCA so only engage when blinkig but not pressed
    if (cp.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"] is not None):
      self.turnSignalStalkState = (
            0
            if cp.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"] == 3
            else int(cp.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"])
        )
    ret.leftBlinker = (cp.vl["GTW_carState"]["BC_indicatorLStatus"] == 1) and (self.turnSignalStalkState == 0) and (self.tap_direction == 1)
    ret.rightBlinker = (cp.vl["GTW_carState"]["BC_indicatorRStatus"] == 1) and (self.turnSignalStalkState == 0) and (self.tap_direction == 2)

    # Seatbelt
    if (self.CP.carFingerprint in [CAR.AP1_MODELX]):
      ret.seatbeltUnlatched = (cp.vl["RCM_status"]["RCM_buckleDriverStatus"] != 1)
    else:
      ret.seatbeltUnlatched = (cp.vl["SDM1"]["SDM_bcklDrivStatus"] != 1)

    #Blindspot
    if (self.CP.carFingerprint != CAR.PREAP_MODELS):
      park_right_blindspot = self.can_define.dv["PARK_status2"]["PARK_sdiBlindSpotRight"].get(int(cp.vl["PARK_status2"]["PARK_sdiBlindSpotRight"])) == "WARNING"
      park_left_blindspot = self.can_define.dv["PARK_status2"]["PARK_sdiBlindSpotLeft"].get(int(cp.vl["PARK_status2"]["PARK_sdiBlindSpotLeft"])) == "WARNING"
      das_right_blindspot = self.can_define.dv["DAS_status"]["DAS_blindSpotRearRight"].get(int(cp_cam.vl["DAS_status"]["DAS_blindSpotRearRight"])) in ["WARNING_LEVEL_2","WARNING_LEVEL_1"]
      das_left_blindspot = self.can_define.dv["DAS_status"]["DAS_blindSpotRearLeft"].get(int(cp_cam.vl["DAS_status"]["DAS_blindSpotRearLeft"])) in ["WARNING_LEVEL_2","WARNING_LEVEL_1"]
    else:
      das_right_blindspot = False
      das_left_blindspot = False
      park_right_blindspot = False
      park_left_blindspot = False
    ret.rightBlindspot = (park_right_blindspot or das_right_blindspot)
    ret.leftBlindspot = (park_left_blindspot or das_left_blindspot)
    # Messages needed by carcontroller
    sw_a = copy.copy(cp.vl["STW_ACTN_RQ"])
    if sw_a is not None:
      self.msg_stw_actn_req = sw_a
      
    #PREAP overrides at the last moment
    if self.CP.carFingerprint in [CAR.PREAP_MODELS]:
      #Message needed for GTW_ESP1
      gtw_esp1 = copy.copy(cp.vl["GTW_ESP1"])
      if gtw_esp1 is not None:
        self.gtw_esp1 = gtw_esp1
        self.gtw_esp1_last_sent_id = self.gtw_esp1["GTW_ESP1Counter"]
      #Pedal Interceptor
      self.prev_pedal_interceptor_state = self.pedal_interceptor_state
      self.prev_pedal_idx = self.pedal_idx
      if self.enablePedal:
        if self.pedalcanzero:
          self.pedal_interceptor_state = cp.vl["GAS_SENSOR"]["STATE"]
          self.pedal_interceptor_value = cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"]
          self.pedal_interceptor_value2 = cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]
          self.pedal_idx = cp.vl["GAS_SENSOR"]["IDX"]
        else:
          self.pedal_interceptor_state = cp_cam.vl["GAS_SENSOR"]["STATE"]
          self.pedal_interceptor_value = cp_cam.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"]
          self.pedal_interceptor_value2 = cp_cam.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]
          self.pedal_idx = cp_cam.vl["GAS_SENSOR"]["IDX"]

      #preAP stuff
      if self.enableHumanLongControl:
        self.enablePedal = (
            self.enablePedalHardware and 
            (
                CruiseState.is_off(self.cruise_state) or 
                self.enablePedalOverCC
            ) and 
            self.CP.openpilotLongitudinalControl
        )
        self.enableACC = (
            not self.enablePedalHardware and
            CruiseState.is_enabled_or_standby(self.cruise_state) and
            self.CP.openpilotLongitudinalControl
        )
        self.enableJustCC = not (
            self.enableACC or
            self.enablePedal
        )
        if self.cruise_buttons == CruiseButtons.MAIN:
          self.cruiseEnabled = not self.enableJustCC
        if self.cruise_buttons == CruiseButtons.CANCEL:
          self.cruiseEnabled = False
          
        ret.cruiseState.enabled = self.cruiseEnabled and (not ret.doorOpen) and (ret.gearShifter == car.CarState.GearShifter.drive) and (not ret.seatbeltUnlatched)
        self.cruiseEnabled = ret.cruiseState.enabled
        ret.cruiseState.available = True
        ret.cruiseState.standstill = ret.standstill
        ret.brakePressed = False
        ret.gasPressed = False
        self.DAS_216_driverOverriding = False
        ret.cruiseState.speed = self.acc_speed_kph * CV.KPH_TO_MS

    return ret

  @staticmethod
  def get_can_parser(CP):
    enablePedal = load_bool_param("TinklaEnablePedal",False)
    pedalcanzero = load_bool_param("TinklaPedalCanZero",False)
    has_ibooster_ecu = load_bool_param("TinklaHasIBooster",False)
    signals = [
      # sig_name, sig_address, default
      ("DI_pedalPos", "DI_torque1", 0),
      ("DI_torqueMotor", "DI_torque1", 0),
      ("DI_brakePedal", "DI_torque2", 0),
      ("StW_AnglHP", "STW_ANGLHP_STAT", 0),
      ("StW_AnglHP_Spd", "STW_ANGLHP_STAT", 0),
      ("DI_cruiseState", "DI_state", 0),
      ("DI_digitalSpeed", "DI_state", 0),
      ("DI_speedUnits", "DI_state", 0),
      ("DI_cruiseSet", "DI_state", 0),
      ("DI_gear", "DI_torque2", 0),
      ("DOOR_STATE_FL", "GTW_carState", 1),
      ("DOOR_STATE_FR", "GTW_carState", 1),
      ("DOOR_STATE_RL", "GTW_carState", 1),
      ("DOOR_STATE_RR", "GTW_carState", 1),
      ("DOOR_STATE_FrontTrunk", "GTW_carState", 1),
      ("BOOT_STATE", "GTW_carState", 1),
      ("BC_indicatorLStatus", "GTW_carState", 1),
      ("BC_indicatorRStatus", "GTW_carState", 1),
      # info for speed limit
      ("UI_mapSpeedLimitUnits","UI_gpsVehicleSpeed",0),
      ("UI_mppSpeedLimit","UI_gpsVehicleSpeed",0),
      ("UI_mapSpeedLimit","UI_driverAssistMapData",0),
      ("UI_roadSign","UI_driverAssistRoadSign",0),
      ("UI_baseMapSpeedLimitMPS","UI_driverAssistRoadSign",0),
      # We copy this whole message when spamming cancel
      ("SpdCtrlLvr_Stat", "STW_ACTN_RQ", 0),
      ("VSL_Enbl_Rq", "STW_ACTN_RQ", 0),
      ("SpdCtrlLvrStat_Inv", "STW_ACTN_RQ", 0),
      ("DTR_Dist_Rq", "STW_ACTN_RQ", 0),
      ("TurnIndLvr_Stat", "STW_ACTN_RQ", 0),
      ("HiBmLvr_Stat", "STW_ACTN_RQ", 0),
      ("WprWashSw_Psd", "STW_ACTN_RQ", 0),
      ("WprWash_R_Sw_Posn_V2", "STW_ACTN_RQ", 0),
      ("StW_Lvr_Stat", "STW_ACTN_RQ", 0),
      ("StW_Cond_Flt", "STW_ACTN_RQ", 0),
      ("StW_Cond_Psd", "STW_ACTN_RQ", 0),
      ("HrnSw_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw00_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw01_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw02_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw03_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw04_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw05_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw06_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw07_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw08_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw09_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw10_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw11_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw12_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw13_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw14_Psd", "STW_ACTN_RQ", 0),
      ("StW_Sw15_Psd", "STW_ACTN_RQ", 0),
      ("WprSw6Posn", "STW_ACTN_RQ", 0),
      ("MC_STW_ACTN_RQ", "STW_ACTN_RQ", 0),
      ("CRC_STW_ACTN_RQ", "STW_ACTN_RQ", 0),
    ]

    checks = [
      # sig_address, frequency
      ("DI_torque1", 100),
      ("DI_torque2", 100),
      ("STW_ANGLHP_STAT", 100),
      ("DI_state", 10),
      ("STW_ACTN_RQ", 10),
      ("GTW_carState", 10),     
    ]

    signals += [
      ("ESP_vehicleSpeed", "ESP_B", 0),
      ("ESP_wheelPulseCountFrL", "ESP_B", 0),
      ("ESP_wheelPulseCountFrR", "ESP_B", 0),
      ("ESP_wheelPulseCountReL", "ESP_B", 0),
      ("ESP_wheelPulseCountReR", "ESP_B", 0),
      ("driverBrakeStatus", "BrakeMessage", 0),
    ]
    checks += [
      ("ESP_B", 50),
      ("BrakeMessage", 50),
    ]
    
    if not (CP.carFingerprint in [CAR.AP1_MODELX]):
      signals += [
        ("SDM_bcklDrivStatus", "SDM1", 0),
      ]

      checks += [
        #("SDM1", 10),
      ]

    if (CP.carFingerprint in [CAR.AP1_MODELX]):
      signals += [
        ("RCM_buckleDriverStatus","RCM_status",0),
      ]

      checks += [
        #("RCM_status",10),
      ]

    if (CP.carFingerprint in [CAR.PREAP_MODELS]):
      signals += [
        ("GTW_ESP1Counter","GTW_ESP1",0),
        ("GTW_hillStartAssistEnabled","GTW_ESP1",0),
        ("GTW_brakeDiscWipeRequest","GTW_ESP1",0),
        ("GTW_brakeFluidLow","GTW_ESP1",0),
        ("GTW_pedalCalRxEnable","GTW_ESP1",0),
        ("GTW_ambientTemperature","GTW_ESP1",0),
        ("GTW_pedalCalTargetCurve","GTW_ESP1",0),
        ("GTW_espModeSwitch","GTW_ESP1",0),
        ("GTW_ESP1Checksum","GTW_ESP1",0),
      ]

      checks += [
        ("GTW_ESP1",1),
      ]

    signals += [
      ("EPAS_handsOnLevel", "EPAS_sysStatus", 0),
      ("EPAS_torsionBarTorque", "EPAS_sysStatus", 0),
      ("EPAS_internalSAS", "EPAS_sysStatus", 0),
      ("EPAS_eacStatus", "EPAS_sysStatus", 1),
      ("EPAS_eacErrorCode", "EPAS_sysStatus", 0),
      ("PARK_sdiBlindSpotRight","PARK_status2",0),
      ("PARK_sdiBlindSpotLeft","PARK_status2",0),
    ]

    checks += [      
      ("EPAS_sysStatus", 25),
      #("PARK_status2",4),
    ]

    if has_ibooster_ecu:
      signals += [
        ("BrakeApplied", "ECU_BrakeStatus", 0),
        ("BrakeOK", "ECU_BrakeStatus", 0),
        ("DriverBrakeApplied", "ECU_BrakeStatus", 0),
        ("BrakePedalPosition", "ECU_BrakeStatus", 0),
      ]

      checks += [
        ("ECU_BrakeStatus", 80)
      ]

    if enablePedal and pedalcanzero:
      signals += [
        ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
        ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0),
        ("STATE", "GAS_SENSOR", 0),
        ("IDX", "GAS_SENSOR", 0),
      ]

      checks += [
        ("GAS_SENSOR", 10)
      ]

    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, 0, enforce_checks=False)

  @staticmethod
  def get_cam_can_parser(CP):
    enablePedal = load_bool_param("TinklaEnablePedal",False)
    pedalcanzero = load_bool_param("TinklaPedalCanZero",False)
    signals = []
    checks = []
    
    if CP.carFingerprint in [CAR.AP1_MODELS, CAR.AP2_MODELS, CAR.AP1_MODELX]:
      signals = [
        # sig_name, sig_address, default
        #we need the steering control counter
        ("DAS_steeringControlCounter", "DAS_steeringControl", 0),
        # We copy this whole message when we change the status for IC
        ("DAS_autopilotState", "DAS_status", 0),
        ("DAS_blindSpotRearLeft", "DAS_status", 0),
        ("DAS_blindSpotRearRight", "DAS_status", 0),
        ("DAS_fusedSpeedLimit", "DAS_status", 0),
        ("DAS_suppressSpeedWarning", "DAS_status", 0),
        ("DAS_summonObstacle", "DAS_status", 0),
        ("DAS_summonClearedGate", "DAS_status", 0),
        ("DAS_visionOnlySpeedLimit", "DAS_status", 0),
        ("DAS_heaterState", "DAS_status", 0),
        ("DAS_forwardCollisionWarning", "DAS_status", 0),
        ("DAS_autoparkReady", "DAS_status", 0),
        ("DAS_autoParked", "DAS_status", 0),
        ("DAS_autoparkWaitingForBrake", "DAS_status", 0),
        ("DAS_summonFwdLeashReached", "DAS_status", 0),
        ("DAS_summonRvsLeashReached", "DAS_status", 0),
        ("DAS_sideCollisionAvoid", "DAS_status", 0),
        ("DAS_sideCollisionWarning", "DAS_status", 0),
        ("DAS_sideCollisionInhibit", "DAS_status", 0),
        ("DAS_csaState", "DAS_status2", 0),
        ("DAS_laneDepartureWarning", "DAS_status", 0),
        ("DAS_fleetSpeedState", "DAS_status", 0),
        ("DAS_autopilotHandsOnState", "DAS_status", 0),
        ("DAS_autoLaneChangeState", "DAS_status", 0),
        ("DAS_summonAvailable", "DAS_status", 0),
        ("DAS_statusCounter", "DAS_status", 0),
        ("DAS_statusChecksum", "DAS_status", 0),
        ("DAS_headlightRequest", "DAS_bodyControls", 0),
        ("DAS_hazardLightRequest", "DAS_bodyControls", 0),
        ("DAS_wiperSpeed", "DAS_bodyControls", 0),
        ("DAS_turnIndicatorRequest", "DAS_bodyControls", 0),
        ("DAS_highLowBeamDecision", "DAS_bodyControls", 0),
        ("DAS_highLowBeamOffReason", "DAS_bodyControls", 0),
        ("DAS_turnIndicatorRequestReason", "DAS_bodyControls", 0),
        ("DAS_bodyControlsCounter", "DAS_bodyControls", 0),
        ("DAS_bodyControlsChecksum", "DAS_bodyControls", 0),
        ("DAS_accSpeedLimit", "DAS_status2", 0),
        ("DAS_pmmObstacleSeverity", "DAS_status2", 0),
        ("DAS_pmmLoggingRequest", "DAS_status2", 0),
        ("DAS_activationFailureStatus", "DAS_status2", 0),
        ("DAS_pmmUltrasonicsFaultReason", "DAS_status2", 0),
        ("DAS_pmmRadarFaultReason", "DAS_status2", 0),
        ("DAS_pmmSysFaultReason", "DAS_status2", 0),
        ("DAS_pmmCameraFaultReason", "DAS_status2", 0),
        ("DAS_ACC_report", "DAS_status2", 0),
        ("DAS_lssState", "DAS_status", 0),
        ("DAS_radarTelemetry", "DAS_status2", 0),
        ("DAS_robState", "DAS_status2", 0),
        ("DAS_driverInteractionLevel", "DAS_status2", 0),
        ("DAS_ppOffsetDesiredRamp", "DAS_status2", 0),
        ("DAS_longCollisionWarning", "DAS_status2", 0),
        ("DAS_status2Counter", "DAS_status2", 0),
        ("DAS_status2Checksum", "DAS_status2", 0),
        ("DAS_eacState", "DAS_pscControl", 0),
        ("DAS_pscParkState", "DAS_pscControl", 0),
      ]

      checks = [
        # sig_address, frequency
        ("DAS_status", 2),
        ("DAS_status2",2),
        ("DAS_pscControl",25),
        ("DAS_bodyControls",2),
        ("DAS_steeringControl",50),

      ]

    if CP.carFingerprint in [CAR.PREAP_MODELS]:
      if enablePedal and not pedalcanzero:
        signals += [
          ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
          ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0),
          ("STATE", "GAS_SENSOR", 0),
          ("IDX", "GAS_SENSOR", 0),
        ]

        checks += [
          ("GAS_SENSOR", 10)
        ]

    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, 2,enforce_checks=False)
