import copy
import crcmod
from opendbc.can.can_define import CANDefine
from common.numpy_fast import clip


class TeslaCAN:
  def __init__(self, dbc_name, packer):
    self.can_define = CANDefine(dbc_name)
    self.packer = packer
    self.crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)

  @staticmethod
  def checksum(msg_id, dat):
    # TODO: get message ID from name instead
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_lane_message(self, lWidth, rLine, lLine, laneRange, curvC0, curvC1, curvC2, curvC3, bus, counter):
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
      "DAS_leftFork" : 0,
      "DAS_rightFork" : 0,
      "DAS_lanesCounter" : counter,
    }
    return self.packer.make_can_msg("DAS_lanes", bus, values)

  def create_lead_car_object_message(self, vType1,relevant1,dx1,vxrel1,dy1,vType2,relevant2,dx2,vxrel2,dy2,bus):
    values = {
      "DAS_objectId" : 0, #0-Lead vehicles
      "DAS_leadVehType" : vType1,
      "DAS_leadVehRelevantForControl" : relevant1,
      "DAS_leadVehDx" : dx1,
      "DAS_leadVehVxRel" : vxrel1,
      "DAS_leadVehDy" : dy1,
      "DAS_leadVehId" : 1,
      "DAS_leadVeh2Type" : vType2,
      "DAS_leadVeh2RelevantForControl" : relevant2,
      "DAS_leadVeh2Dx" : dx2,
      "DAS_leadVeh2VxRel" : vxrel2,
      "DAS_leadVeh2Dy" : dy2,
      "DAS_leadVeh2Id" : 2,
    }
    return self.packer.make_can_msg("DAS_object", bus, values)
  
  def create_warning0_message(self,msg_das_warningMsg0):
    values = copy.copy(msg_das_warningMsg0)

  def create_body_controls_message(self,msg_das_body_controls,turn,hazard):
    if msg_das_body_controls != None:
      values = copy.copy(msg_das_body_controls)
    else:
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


  def create_telemetry_road_info(self, rLineType, rLineQual, rLineColor, lLineType, lLineQual, lLineColor, alcaState, bus):
    #alcaState -1 alca to left, 1 alca to right, 0 no alca now
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
      "DAS_steeringControlType": 1 if enabled else 0,
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


  def create_action_request(self, msg_stw_actn_req, cancel, bus, counter):
    values = copy.copy(msg_stw_actn_req)

    if cancel:
      values["SpdCtrlLvr_Stat"] = 1
      #for counter take the last received countr and increment by 1
      values["MC_STW_ACTN_RQ"] = counter

    data = self.packer.make_can_msg("STW_ACTN_RQ", bus, values)[2]
    values["CRC_STW_ACTN_RQ"] = self.crc(data[:7])
    return self.packer.make_can_msg("STW_ACTN_RQ", bus, values)
