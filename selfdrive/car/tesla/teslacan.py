import copy
import crcmod
from opendbc.can.can_define import CANDefine
from common.numpy_fast import clip
from ctypes import create_string_buffer
import struct
from selfdrive.config import Conversions as CV
from selfdrive.car import make_can_msg
from selfdrive.car.tesla.values import CarControllerParams, CAN_CHASSIS, CAN_POWERTRAIN

class TeslaCAN: 
  def __init__(self, dbc_name, packer, pt_packer):
    self.can_define = CANDefine(dbc_name)
    self.packer = packer
    self.pt_packer = pt_packer
    self.crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)

  @staticmethod
  def checksum(msg_id, dat):
    # TODO: get message ID from name instead
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_lane_message(self, lWidth, rLine, lLine, laneRange, curvC0, curvC1, curvC2, curvC3, lLane2,rLane2,bus, counter):
    values = {
      "DAS_leftLaneExists" : lLine,
      "DAS_rightLaneExists" : rLine,
      "DAS_virtualLaneWidth" : lWidth,
      "DAS_virtualLaneViewRange" : laneRange,
      "DAS_virtualLaneC0" : curvC0,
      "DAS_virtualLaneC1" : curvC1,
      "DAS_virtualLaneC2" : curvC2,
      "DAS_virtualLaneC3" : curvC3,
      "DAS_leftLineUsage" : lLine * 2, 
      "DAS_rightLineUsage" : rLine * 2,
      "DAS_leftFork" : lLane2,
      "DAS_rightFork" : rLane2,
      "DAS_lanesCounter" : counter,
    }
    return self.packer.make_can_msg("DAS_lanes", bus, values)

  def create_lead_car_object_message(self, objectId, vType1,vId1,relevant1,dx1,vxrel1,dy1,vType2,vId2,relevant2,dx2,vxrel2,dy2,bus):
    values = {
      "DAS_objectId" : objectId, #0-Lead vehicles
      "DAS_leadVehType" : vType1,
      "DAS_leadVehRelevantForControl" : relevant1,
      "DAS_leadVehDx" : dx1,
      "DAS_leadVehVxRel" : vxrel1,
      "DAS_leadVehDy" : dy1,
      "DAS_leadVehId" : vId1,
      "DAS_leadVeh2Type" : vType2,
      "DAS_leadVeh2RelevantForControl" : relevant2,
      "DAS_leadVeh2Dx" : dx2,
      "DAS_leadVeh2VxRel" : vxrel2,
      "DAS_leadVeh2Dy" : dy2,
      "DAS_leadVeh2Id" : vId2,
    }
    return self.packer.make_can_msg("DAS_object", bus, values)

  def create_body_controls_message(self,turn,hazard,bus,counter):
    values = {
      "DAS_headlightRequest" : 0, 
      "DAS_hazardLightRequest" : 0,
      "DAS_wiperSpeed" : 0, 
      "DAS_turnIndicatorRequest" : 0, 
      "DAS_highLowBeamDecision" : 3, 
      "DAS_highLowBeamOffReason" : 5, 
      "DAS_turnIndicatorRequestReason" : 0, 
      "DAS_bodyControlsCounter" : 1 ,
      "DAS_bodyControlsChecksum" : 0, 
    }
    values["DAS_hazardLightRequest"] = hazard
    values["DAS_turnIndicatorRequest"] = turn #0-off, 1-left 2-right
    if turn > 0:
      values["DAS_turnIndicatorRequestReason"] = 1
    else:
      values["DAS_turnIndicatorRequestReason"] = 0

    return self.packer.make_can_msg("DAS_bodyControls", bus, values)


  def create_telemetry_road_info(self, lLineQualRaw, rLineQualRaw, alcaState, bus):
    #alcaState -1 alca to left, 1 alca to right, 0 no alca now
    rLineType = 7
    rLineColor = 0
    rLineQual = 0
    if rLineQualRaw > 0.15:
      rLineType = 3
      rLineColor = 1
      if rLineQualRaw > 0.75:
        rLineQual = 3
      elif rLineQualRaw > 0.5:
        rLineQual = 2
      else:
        rLineQual = 1
    lLineType = 7
    lLineColor = 0
    lLineQual = 0
    if lLineQualRaw > 0.45:
      lLineType = 3
      lLineColor = 1
      if lLineQualRaw > 0.75:
        lLineQual = 3
      elif lLineQualRaw > 0.5:
        lLineQual = 2
      else:
        lLineQual = 1
    values = {
      "DAS_telemetryMultiplexer" : 0,
      "DAS_telLeftLaneType" : lLineType, #0-undecided, 1-solid, 2-road edge, 3-dashed 4-double 5-botts dots 6-barrier
      "DAS_telRightLaneType" : rLineType, #0-undecided, 1-solid, 2-road edge, 3-dashed 4-double 5-botts dots 6-barrier
      "DAS_telLeftMarkerQuality" : lLineQual, # 0  LOWEST, 1 LOW, 2 MEDIUM, 3 HIGH
      "DAS_telRightMarkerQuality" : rLineQual, # 0  LOWEST, 1 LOW, 2 MEDIUM, 3 HIGH
      "DAS_telLeftMarkerColor" : lLineColor, # 0 UNKNOWN, 1 WHITE, 2 YELLOW, 3 BLUE
      "DAS_telRightMarkerColor" : rLineColor, # 0 UNKNOWN, 1 WHITE, 2 YELLOW, 3 BLUE
      "DAS_telLeftLaneCrossing" : 0 if alcaState != -1 else 1, #0 NOT CROSSING, 1 CROSSING
      "DAS_telRightLaneCrossing" : 0 if alcaState != 1 else 1,#0 NOT CROSSING, 1 CROSSING
    }
    return self.packer.make_can_msg("DAS_telemetry", bus, values)

  def create_steering_control(self, angle, enabled, bus, counter):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0, #0-NONE, 1-ANGLE, 2-LKA, 3-Emergency LKA
      "DAS_steeringControlCounter": counter,
      "DAS_steeringControlChecksum": 0,
    }
    return self.packer.make_can_msg("DAS_steeringControl", bus, values)

  def create_ap1_long_control(self, speed, accel_limits, jerk_limits, bus, counter):
    accState = 0
    if speed == 0:
      accState = 3
    else:
      accState = 4
    values = {
      "DAS_setSpeed" :  clip(speed*3.6,0,410), #kph
      "DAS_accState" :  accState, # 4-ACC ON, 3-HOLD, 0-CANCEL
      "DAS_aebEvent" :  0, # 0 - AEB NOT ACTIVE
      "DAS_jerkMin" :  clip(jerk_limits[0],-7.67,0), #m/s^3 -8.67,0
      "DAS_jerkMax" :  clip(jerk_limits[1],0,7.67), #m/s^3 0,8.67
      "DAS_accelMin" : clip(accel_limits[0],-12,3.44), #m/s^2 -15,5.44
      "DAS_accelMax" : clip(accel_limits[1],-12,3.44), #m/s^2 -15,5.44
      "DAS_controlCounter": counter,
      "DAS_controlChecksum" : 0,
    }
    return self.packer.make_can_msg("DAS_control", bus, values)

  def create_ap2_long_control(self, speed, accel_limits, jerk_limits, bus, counter):
    locRequest = 0
    if speed == 0:
      locRequest = 3
    else:
      locRequest = 1
    values = {
      "DAS_locMode" : 1, # 1- NORMAL
      "DAS_locState" : 0, # 0-HEALTHY
      "DAS_locRequest" : locRequest, # 0-IDLE,1-FORWARD,2-REVERSE,3-HOLD,4-PARK
      "DAS_locJerkMin" : clip(jerk_limits[0],-7.67,0), #m/s^3 -8.67,0
      "DAS_locJerkMax" : clip(jerk_limits[1],0,7.67), #m/s^3 0,8.67
      "DAS_locSpeed" : clip(speed*3.6,0,200), #kph
      "DAS_locAccelMin" : clip(accel_limits[0],-12,3.44), #m/s^2 -15,5.44
      "DAS_locAccelMax" : clip(accel_limits[1],-12,3.44), #m/s^2 -15,5.44
      "DAS_longControlCounter" : counter, #
      "DAS_longControlChecksum" : 0, #
    }
    return self.packer.make_can_msg("DAS_longControl", bus, values)

  def create_das_warningMatrix0 (self, DAS_canErrors, DAS_025_steeringOverride, DAS_notInDrive, bus):
    msg_id = 0x329
    msg_len = 8
    msg = create_string_buffer(msg_len)
    struct.pack_into("BBBBBBBB", msg, 0,
      0,0,0,DAS_025_steeringOverride + (DAS_canErrors << 7),0,(DAS_notInDrive << 7),0,0)
    return [msg_id, 0, msg.raw, bus]

  def create_das_warningMatrix1 (self, bus):
    msg_id = 0x369
    msg_len = 8
    msg = create_string_buffer(msg_len)
    struct.pack_into("BBBBBBBB", msg, 0,
      0,0,0,0,0,0,0,0)
    return [msg_id, 0, msg.raw, bus]

  def create_das_warningMatrix3 (self, DAS_gas_to_resume, DAS_211_accNoSeatBelt, DAS_202_noisyEnvironment , DAS_206_apUnavailable, DAS_207_lkasUnavailable,
    DAS_219_lcTempUnavailableSpeed, DAS_220_lcTempUnavailableRoad, DAS_221_lcAborting, DAS_222_accCameraBlind,
    DAS_208_rackDetected, DAS_w216_driverOverriding, stopSignWarning, stopLightWarning, bus):
    msg_id = 0x349
    msg_len = 8
    msg = create_string_buffer(msg_len)
    struct.pack_into("BBBBBBBB", msg, 0,
      (DAS_gas_to_resume << 1) + (stopSignWarning << 3) + (stopLightWarning << 4),
      (DAS_202_noisyEnvironment << 1) + (DAS_206_apUnavailable << 5) + (DAS_207_lkasUnavailable << 6) + (DAS_208_rackDetected << 7),
      (DAS_211_accNoSeatBelt << 2) + (DAS_w216_driverOverriding << 7),
      (DAS_219_lcTempUnavailableSpeed << 2) + (DAS_220_lcTempUnavailableRoad << 3) + (DAS_221_lcAborting << 4) + (DAS_222_accCameraBlind << 5),
      0,0,0,0)
    return [msg_id, 0, msg.raw, bus]
    

  def create_das_status (self, DAS_op_status, DAS_collision_warning,
    DAS_ldwStatus, DAS_hands_on_state, DAS_alca_state, 
    blindSpotLeft, blindSpotRight,
    DAS_speed_limit_kph, bus, counter):
    values = {
      "DAS_autopilotState" : DAS_op_status,
      "DAS_blindSpotRearLeft" : 1 if blindSpotLeft else 0,
      "DAS_blindSpotRearRight" : 1 if blindSpotRight else 0,
      "DAS_fusedSpeedLimit" : DAS_speed_limit_kph,
      "DAS_suppressSpeedWarning" : 1,
      "DAS_summonObstacle" : 0,
      "DAS_summonClearedGate" : 0,
      "DAS_visionOnlySpeedLimit" : DAS_speed_limit_kph,
      "DAS_heaterState" : 0,
      "DAS_forwardCollisionWarning" : DAS_collision_warning,
      "DAS_autoparkReady" : 0,
      "DAS_autoParked" : 0,
      "DAS_autoparkWaitingForBrake" : 0,
      "DAS_summonFwdLeashReached" : 0,
      "DAS_summonRvsLeashReached" : 0,
      "DAS_sideCollisionAvoid" : 0,
      "DAS_sideCollisionWarning" : 0,
      "DAS_sideCollisionInhibit" : 0,
      "DAS_lssState" : 0, #0-FAULT 
      "DAS_laneDepartureWarning" : DAS_ldwStatus,
      "DAS_fleetSpeedState" : 0,
      "DAS_autopilotHandsOnState" : DAS_hands_on_state, # 3 quiet, 5 with alerts
      "DAS_autoLaneChangeState" : DAS_alca_state,
      "DAS_summonAvailable" : 0,
      "DAS_statusCounter" : counter,
      "DAS_statusChecksum" : 0,
    }
    return self.packer.make_can_msg("DAS_status", bus, values)

  def create_das_status2(self, DAS_acc_speed_limit, fcw, bus, counter):
    fcw_sig = 0x0F if fcw == 0 else 0x01
    values = {
      "DAS_accSpeedLimit" : DAS_acc_speed_limit,
      "DAS_pmmObstacleSeverity" : 0,
      "DAS_pmmLoggingRequest" : 0,
      "DAS_activationFailureStatus" : 0,
      "DAS_pmmUltrasonicsFaultReason" : 0,
      "DAS_pmmRadarFaultReason" : 0,
      "DAS_pmmSysFaultReason" : 0,
      "DAS_pmmCameraFaultReason" : 0,
      "DAS_ACC_report" : 1, #ACC_report_target_CIPV
      "DAS_csaState" : 2, #CSA_EXTERNAL_STATE_AVAILABLE
      "DAS_radarTelemetry" : 1, #normal
      "DAS_robState" : 2, #active
      "DAS_driverInteractionLevel" : 0, 
      "DAS_ppOffsetDesiredRamp" : 0x80,
      "DAS_longCollisionWarning" : fcw_sig,
      "DAS_status2Counter" : counter,
      "DAS_status2Checksum" : 0,
    }
    return self.packer.make_can_msg("DAS_status2", bus, values)

  def create_action_request(self, msg_stw_actn_req, button_to_press, bus, counter):
    values = copy.copy(msg_stw_actn_req)

    values["SpdCtrlLvr_Stat"] = button_to_press
    values["MC_STW_ACTN_RQ"] = counter

    data = self.packer.make_can_msg("STW_ACTN_RQ", bus, values)[2]
    values["CRC_STW_ACTN_RQ"] = self.crc(data[:7])
    return self.packer.make_can_msg("STW_ACTN_RQ", bus, values)

  def create_radar_VIN_msg(self, radarId, radarVIN, radarCAN, radarTriggerMessage,
      useRadar, radarPosition, radarEpasType ):
    msg_id = 0x560
    msg_len = 8
    msg = create_string_buffer(msg_len)
    if radarId == 0:
      struct.pack_into(
          "BBBBBBBB",
          msg,
          0,
          radarId,
          radarCAN,
          useRadar + (radarPosition << 1) + (radarEpasType << 3),
          int((radarTriggerMessage >> 8) & 0xFF),
          (radarTriggerMessage & 0xFF),
          ord(radarVIN[0]),
          ord(radarVIN[1]),
          ord(radarVIN[2]),
      )
    if radarId == 1:
      struct.pack_into(
          "BBBBBBBB",
          msg,
          0,
          radarId,
          ord(radarVIN[3]),
          ord(radarVIN[4]),
          ord(radarVIN[5]),
          ord(radarVIN[6]),
          ord(radarVIN[7]),
          ord(radarVIN[8]),
          ord(radarVIN[9]),
      )
    if radarId == 2:
      struct.pack_into(
          "BBBBBBBB",
          msg,
          0,
          radarId,
          ord(radarVIN[10]),
          ord(radarVIN[11]),
          ord(radarVIN[12]),
          ord(radarVIN[13]),
          ord(radarVIN[14]),
          ord(radarVIN[15]),
          ord(radarVIN[16]),
      )
    return [msg_id, 0, msg.raw, radarCAN]

  def create_fake_DAS_msg(
    self,
    speed_control_enabled,
    speed_override,
    apUnavailable,
    collision_warning,
    op_status,
    acc_speed_kph,
    turn_signal_needed,
    forward_collission_warning,
    adaptive_cruise,
    hands_on_state,
    cc_state,
    pcc_available,
    alca_state,
    acc_speed_limit,  # IC cruise speed, kph or mph
    legal_speed_limit,
    apply_angle,
    enable_steer_control,
    park_brake_request,
    bus,
  ):
    msg_id = 0x659  # we will use DAS_udsRequest to send this info to IC
    msg_len = 8
    msg = create_string_buffer(msg_len)
    units_included = 1
    c_apply_steer = int(
        ((int(apply_angle * 10 + 0x4000)) & 0x7FFF) + (enable_steer_control << 15)
    )
    dat = [
        int(
            (speed_control_enabled << 7)
            + (speed_override << 6)
            + (apUnavailable << 5)
            + (collision_warning << 4)
            + op_status
        ),
        int(acc_speed_kph),
        int(
            (turn_signal_needed << 6)
            + (units_included << 5)
            + (forward_collission_warning << 4)
            + (adaptive_cruise << 3)
            + hands_on_state
        ),
        int((cc_state << 6) + (pcc_available << 5) + alca_state),
        int(
            acc_speed_limit + 0.5
        ),  # IC rounds current speed, so we need to round cruise speed the same way
        int(
            (legal_speed_limit & 0x1F) + ((park_brake_request << 5) & 0x20)
        ),  # positions 7 and 6 not used yet
        int(c_apply_steer & 0xFF),
        int((c_apply_steer >> 8) & 0xFF)
    ]
    return make_can_msg(0x659, bytes(dat), bus)

  def create_pedal_command_msg(self,accelCommand, enable, idx, pedalcan):
      """Create GAS_COMMAND (0x551) message to comma pedal"""
      msg_id = 0x551
      msg_len = 6
      msg = create_string_buffer(msg_len)
      m1 = 0.050796813
      m2 = 0.101593626
      d = -22.85856576
      if enable == 1:
          int_accelCommand = int((accelCommand - d) / m1)
          int_accelCommand2 = int((accelCommand - d) / m2)
      else:
          int_accelCommand = 0
          int_accelCommand2 = 0
      msg = create_string_buffer(msg_len)
      struct.pack_into(
          "BBBBB",
          msg,
          0,
          int((int_accelCommand >> 8) & 0xFF),
          int_accelCommand & 0xFF,
          int((int_accelCommand2 >> 8) & 0xFF),
          int_accelCommand2 & 0xFF,
          ((enable << 7) + idx) & 0xFF,
      )
      struct.pack_into("B", msg, msg_len - 1, self.checksum(msg_id,msg.raw))
      return [msg_id, 0, msg.raw, pedalcan]

  #BBTODO: login for long control with ibooster
  def create_longitudinal_commands(self, acc_state, speed, min_accel, max_accel, cnt, fingerprint):
    messages = []
    values = {
      "DAS_setSpeed": speed * CV.MS_TO_KPH,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": min_accel,
      "DAS_accelMax": max_accel,
      "DAS_controlCounter": (cnt % 8),
      "DAS_controlChecksum": 0,
    }

    #BBTODO: don't think we need to send on both.... gtw should forward
    #also change counter and checksum so we mod on forward
    for packer, bus in [(self.packer, CAN_CHASSIS), (self.pt_packer, CAN_POWERTRAIN[fingerprint])]:
      if packer:
        data = packer.make_can_msg("DAS_control", bus, values)[2]
        values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
        messages.append(packer.make_can_msg("DAS_control", bus, values))
    return messages
