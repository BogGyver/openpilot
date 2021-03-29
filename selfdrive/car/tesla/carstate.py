import copy
from cereal import car,log
from selfdrive.car.tesla.values import DBC, GEAR_MAP, DOORS, BUTTONS, CAR
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.tesla.CFG_module import read_config_file

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    self.button_states = {button.event_type: False for button in BUTTONS}
    self.can_define = CANDefine(DBC[CP.carFingerprint]['chassis'])

    # Needed by carcontroller
    self.msg_stw_actn_req = None
    self.msg_autopilot_status = None
    self.msg_das_warningMsg0 = None
    self.msg_das_warningMsg1 = None
    self.msg_das_warningMsg3 = None
    self.msg_das_body_controls = None

    self.hands_on_level = 0
    self.das_steeringControl_counter = -1
    self.das_status_counter = -1
    self.steer_warning = None
    self.angle_steers = 0
    # 0 = off, 1 = indicate left (stalk down), 2 = indicate right (stalk up)
    self.turn_signal_stalk_state = 0 

    #alca info
    self.prev_alca_pre_engage = False
    self.alca_pre_engage = False
    self.alca_engaged = False
    self.alca_direction = 0
    self.alca_need_engagement = False

    #start config section
    self.forcePedalOverCC = False
    self.enableHSO = True
    self.autoStartAlcaDelay = 0
    self.enableDasEmulation = False
    self.enableRadarEmulation = False
    self.hasTeslaIcIntegration = False
    self.useTeslaRadar = False
    self.radarVIN = '"                 "'
    self.radarOffset = 0.0
    self.radarEpasType = 0
    self.radarPosition = 0
    self.hsoNumbPeriod = 1.5
    self.usesApillarHarness = False
    read_config_file(self)
    #end config section

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = (ret.vEgo < 0.1)

    # Gas pedal
    ret.gas = cp.vl["DI_torque1"]["DI_pedalPos"] / 100.0
    ret.gasPressed = (ret.gas > 0)

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = bool(cp.vl["BrakeMessage"]["driverBrakeStatus"] != 1)

    # Steering wheel
    if (self.CP.carFingerprint == CAR.PREAP_MODELS):
      self.hands_on_level = cp_cam.vl["EPAS_sysStatus"]["EPAS_handsOnLevel"]
      self.steer_warning = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(cp_cam.vl["EPAS_sysStatus"]["EPAS_eacErrorCode"]), None)
      steer_status = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(cp_cam.vl["EPAS_sysStatus"]["EPAS_eacStatus"]), None)

      ret.steeringAngleDeg = -cp_cam.vl["EPAS_sysStatus"]["EPAS_internalSAS"]
      self.angle_steers = ret.steeringAngleDeg
      ret.steeringTorque = -cp_cam.vl["EPAS_sysStatus"]["EPAS_torsionBarTorque"]
    else:
      self.hands_on_level = cp.vl["EPAS_sysStatus"]["EPAS_handsOnLevel"]
      self.steer_warning = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(cp.vl["EPAS_sysStatus"]["EPAS_eacErrorCode"]), None)
      steer_status = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(cp.vl["EPAS_sysStatus"]["EPAS_eacStatus"]), None)

      ret.steeringAngleDeg = -cp.vl["EPAS_sysStatus"]["EPAS_internalSAS"]
      self.angle_steers = ret.steeringAngleDeg
      ret.steeringTorque = -cp.vl["EPAS_sysStatus"]["EPAS_torsionBarTorque"]

    ret.steeringRateDeg = -cp.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"] # This is from a different angle sensor, and at different rate

    ret.steeringPressed = (self.hands_on_level > 0)
    ret.steerError = steer_status in ["EAC_FAULT"] #inhibited is not an error, "EAC_INHIBITED"]
    ret.steerWarning = self.steer_warning in ["EAC_ERROR_MAX_SPEED", "EAC_ERROR_MIN_SPEED", "EAC_ERROR_TMP_FAULT", "SNA"]  # TODO: not sure if this list is complete
    #Trick the alca if autoStartAlcaDelay is set
    if (self.autoStartAlcaDelay > 0) and (self.alca_need_engagement):
      ret.steeringPressed = True
      if self.alca_direction == log.LateralPlan.LaneChangeDirection.left:
        ret.steeringTorque = 0.1
      if self.alca_direction == log.LateralPlan.LaneChangeDirection.right:
        ret.steeringTorque = -0.1

    # Cruise state
    if (not self.CP.carFingerprint == CAR.PREAP_MODELS):
      autopilot_status = self.can_define.dv["AutopilotStatus"]["autopilotStatus"].get(int(cp_cam.vl["AutopilotStatus"]["autopilotStatus"]), None)
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp.vl["DI_state"]["DI_speedUnits"]), None)

    acc_enabled = (cruise_state in ["ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL"])
    autopilot_enabled = (autopilot_status in ["ACTIVE_1", "ACTIVE_2", "ACTIVE_NAVIGATE_ON_AUTOPILOT"])

    ret.cruiseState.enabled = acc_enabled and not autopilot_enabled
    if speed_units == "KPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS
    elif speed_units == "MPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS
    ret.cruiseState.available = ((cruise_state == "STANDBY") or ret.cruiseState.enabled)
    ret.cruiseState.standstill = (cruise_state == "STANDSTILL")

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_torque2"]["DI_gear"].get(int(cp.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

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
    ret.leftBlinker = (cp.vl["GTW_carState"]["BC_indicatorLStatus"] == 1)
    ret.rightBlinker = (cp.vl["GTW_carState"]["BC_indicatorRStatus"] == 1)
    self.turn_signal_stalk_state = (
            0
            if cp.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"] == 3
            else int(cp.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"])
        )

    # Seatbelt
    ret.seatbeltUnlatched = (cp.vl["SDM1"]["SDM_bcklDrivStatus"] != 1)

    # TODO: blindspot

    # Messages needed by carcontroller
    self.msg_stw_actn_req = copy.copy(cp.vl["STW_ACTN_RQ"])
    if (nself.CP.carFingerprint != CAR.PREAP_MODELS):
      self.msg_autopilot_status = copy.copy(cp_cam.vl["AutopilotStatus"])
      self.msg_das_body_controls = copy.copy(cp_cam.vl["DAS_bodyControls"])
    #BB will need telemetry (ID) and bodyControl (full) also for IC integration
    #BB for long control we will need also to modify status2 to integrate with IC

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("ESP_vehicleSpeed", "ESP_B", 0),
      ("DI_pedalPos", "DI_torque1", 0),
      ("DI_brakePedal", "DI_torque2", 0),
      ("StW_AnglHP", "STW_ANGLHP_STAT", 0),
      ("StW_AnglHP_Spd", "STW_ANGLHP_STAT", 0),
      ("EPAS_handsOnLevel", "EPAS_sysStatus", 0),
      ("EPAS_torsionBarTorque", "EPAS_sysStatus", 0),
      ("EPAS_internalSAS", "EPAS_sysStatus", 0),
      ("EPAS_eacStatus", "EPAS_sysStatus", 1),
      ("EPAS_eacErrorCode", "EPAS_sysStatus", 0),
      ("DI_cruiseState", "DI_state", 0),
      ("DI_digitalSpeed", "DI_state", 0),
      ("DI_speedUnits", "DI_state", 0),
      ("DI_gear", "DI_torque2", 0),
      ("DOOR_STATE_FL", "GTW_carState", 1),
      ("DOOR_STATE_FR", "GTW_carState", 1),
      ("DOOR_STATE_RL", "GTW_carState", 1),
      ("DOOR_STATE_RR", "GTW_carState", 1),
      ("DOOR_STATE_FrontTrunk", "GTW_carState", 1),
      ("BOOT_STATE", "GTW_carState", 1),
      ("BC_indicatorLStatus", "GTW_carState", 1),
      ("BC_indicatorRStatus", "GTW_carState", 1),
      ("SDM_bcklDrivStatus", "SDM1", 0),
      ("driverBrakeStatus", "BrakeMessage", 0),

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
      ("ESP_B", 50),
      ("DI_torque1", 100),
      ("DI_torque2", 100),
      ("STW_ANGLHP_STAT", 100),
      ("EPAS_sysStatus", 25),
      ("DI_state", 10),
      ("STW_ACTN_RQ", 10),
      ("GTW_carState", 10),
      ("SDM1", 10),
      ("BrakeMessage", 50),
    ]

    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      #we need the steering control counter
      ("DAS_steeringControlCounter", "DAS_steeringControl", 0),
      # We copy this whole message when we change the status for IC
      ("autopilotStatus", "AutopilotStatus", 0),
      ("DAS_blindSpotRearLeft", "AutopilotStatus", 0),
      ("DAS_blindSpotRearRight", "AutopilotStatus", 0),
      ("DAS_fusedSpeedLimit", "AutopilotStatus", 0),
      ("DAS_suppressSpeedWarning", "AutopilotStatus", 0),
      ("DAS_summonObstacle", "AutopilotStatus", 0),
      ("DAS_summonClearedGate", "AutopilotStatus", 0),
      ("DAS_visionOnlySpeedLimit", "AutopilotStatus", 0),
      ("DAS_heaterState", "AutopilotStatus", 0),
      ("DAS_forwardCollisionWarning", "AutopilotStatus", 0),
      ("DAS_autoparkReady", "AutopilotStatus", 0),
      ("DAS_autoParked", "AutopilotStatus", 0),
      ("DAS_autoparkWaitingForBrake", "AutopilotStatus", 0),
      ("DAS_summonFwdLeashReached", "AutopilotStatus", 0),
      ("DAS_summonRvsLeashReached", "AutopilotStatus", 0),
      ("DAS_sideCollisionAvoid", "AutopilotStatus", 0),
      ("DAS_sideCollisionWarning", "AutopilotStatus", 0),
      ("DAS_sideCollisionInhibit", "AutopilotStatus", 0),
      ("DAS_csaState", "AutopilotStatus", 0),
      ("DAS_laneDepartureWarning", "AutopilotStatus", 0),
      ("DAS_fleetSpeedState", "AutopilotStatus", 0),
      ("DAS_autopilotHandsOnState", "AutopilotStatus", 0),
      ("DAS_autoLaneChangeState", "AutopilotStatus", 0),
      ("DAS_summonAvailable", "AutopilotStatus", 0),
      ("DAS_statusCounter", "AutopilotStatus", 0),
      ("DAS_statusChecksum", "AutopilotStatus", 0),
      ("DAS_headlightRequest", "DAS_bodyControls", 0),
      ("DAS_hazardLightRequest", "DAS_bodyControls", 0),
      ("DAS_wiperSpeed", "DAS_bodyControls", 0),
      ("DAS_turnIndicatorRequest", "DAS_bodyControls", 0),
      ("DAS_highLowBeamDecision", "DAS_bodyControls", 0),
      ("DAS_highLowBeamOffReason", "DAS_bodyControls", 0),
      ("DAS_turnIndicatorRequestReason", "DAS_bodyControls", 0),
      ("DAS_bodyControlsCounter", "DAS_bodyControls", 0),
      ("DAS_bodyControlsChecksum", "DAS_bodyControls", 0),
    ]
    checks = [
      # sig_address, frequency
      ("AutopilotStatus", 2),
    ]
    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, 2)
