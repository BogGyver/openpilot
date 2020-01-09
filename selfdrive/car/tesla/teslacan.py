import struct
from ctypes import create_string_buffer
from common.numpy_fast import clip

IC_LANE_SCALE = 2.0

def add_tesla_crc(msg,msg_len):
  """Calculate CRC8 using 1D poly, FF start, FF end"""
  crc_lookup = [0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB, 
0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76, 
0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C, 
0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1, 
0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8, 
0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65, 
0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F, 
0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2, 
0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D, 
0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50, 
0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A, 
0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7, 
0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E, 
0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43, 
0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09, 
0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4]
  crc = 0xFF
  for x in range(0,msg_len,1):
    crc = crc_lookup[crc ^ ord(msg[x])]
  crc = crc ^ 0xFF 
  return crc


def add_tesla_checksum(msg_id,msg):
 """Calculates the checksum for the data part of the Tesla message"""
 checksum = ((msg_id) & 0xFF) + int((msg_id >> 8) & 0xFF)
 for i in range(0,len(msg),1):
  checksum = (checksum + ord(msg[i])) & 0xFF
 return checksum


def create_pedal_command_msg(accelCommand, enable, idx):
  """Create GAS_COMMAND (0x551) message to comma pedal"""
  msg_id = 0x551
  msg_len = 6
  msg = create_string_buffer(msg_len)
  m1 = 0.050796813
  m2 = 0.101593626
  d = -22.85856576
  if enable == 1:
    int_accelCommand = int((accelCommand - d)/m1)
    int_accelCommand2 = int((accelCommand - d)/m2)
  else:
    int_accelCommand = 0
    int_accelCommand2 = 0
  msg = create_string_buffer(msg_len)
  struct.pack_into('BBBBB', msg, 0, int((int_accelCommand >> 8) & 0xFF), int_accelCommand & 0xFF, \
      int((int_accelCommand2 >> 8) & 0xFF), int_accelCommand2 & 0XFF,((enable << 7) + idx) & 0xFF)
  struct.pack_into('B', msg, msg_len-1, add_tesla_checksum(msg_id,msg))
  return [msg_id, 0, msg.raw, 2]    

def create_enabled_eth_msg(status):
  msg_id = 0x018
  msg_len = 1
  msg = create_string_buffer(msg_len)
  struct.pack_into('B', msg, 0, status)
  return [msg_id, 0, msg.raw, 0]
  
def create_fake_IC_msg():
  msg_id = 0x649
  msg_len = 8
  msg = create_string_buffer(msg_len)
  struct.pack_into('BBBBBBBB', msg, 0, 0xFF,0xFF,0x01,0x02,0x03,0x04,0xFF,0x00)
  return [msg_id, 0, msg.raw, 0]

def create_radar_VIN_msg(radarId,radarVIN,radarCAN,radarTriggerMessage,useRadar,radarPosition,radarEpasType):
  msg_id = 0x560
  msg_len = 8
  msg = create_string_buffer(msg_len)
  if radarId == 0:
    struct.pack_into('BBBBBBBB', msg, 0, radarId, radarCAN, useRadar + (radarPosition << 1) + (radarEpasType << 3),int((radarTriggerMessage >> 8) & 0xFF),(radarTriggerMessage & 0xFF),ord(radarVIN[0]),ord(radarVIN[1]),ord(radarVIN[2]))
  if radarId == 1:
    struct.pack_into('BBBBBBBB', msg, 0, radarId, ord(radarVIN[3]), ord(radarVIN[4]),ord(radarVIN[5]),ord(radarVIN[6]),ord(radarVIN[7]),ord(radarVIN[8]),ord(radarVIN[9]))
  if radarId == 2:
    struct.pack_into('BBBBBBBB', msg, 0, radarId, ord(radarVIN[10]), ord(radarVIN[11]),ord(radarVIN[12]),ord(radarVIN[13]),ord(radarVIN[14]),ord(radarVIN[15]),ord(radarVIN[16]))
  return [msg_id, 0, msg.raw, 0]

def create_DAS_LR_object_msg(lane,v1Class,v1Id,v1Dx,v1Dy,v1V,v2Class,v2Id,v2Dx,v2Dy,v2V):
  msg_id = 0x559
  msg_len = 8
  msg = create_string_buffer(msg_len)
  important1 = 0
  important2 = 0
  if (v1Dx > 0) and (v1Id >= 0):
    if lane == 0:
      important1 = 1
    v1Class += 1
    if v1Class == 4:
      v1Class = 5
  if (v2Dx > 0) and (v2Id >= 0):
    #important2 = 1
    v2Class += 1
    if v2Class == 4:
      v2Class = 5
  if (v1Dx > 0):
    v1x = int(clip(v1Dx,0,127)/0.5/IC_LANE_SCALE) & 0xFF
    v1y = int((clip(v1Dy,-22.,22.) + 22.05)/0.35) & 0x7F
    v1v = 0x0F
    if v1Dx > 0:
      v1v = int((clip(v1V,-30,26)/IC_LANE_SCALE + 30)/4) & 0x0F
  else:
    v1x = 0xFF
    v1y = 0x7F
    v1v = 0x0F
    important1 = 0
    v1Class = 0
  if (v2Dx > 0):
    v2x = int(clip(v2Dx,0,127)/0.5/IC_LANE_SCALE) & 0xFF
    v2y = int((clip(v2Dy,-22.,22.) + 22.05)/0.35) & 0x7F
    v2v = 0x0F
    if v2Dx > 0:
      v2v = int((clip(v2V,-30,26)/IC_LANE_SCALE + 30)/4) & 0x0F
  else:
    v2x = 0xFF
    v2y = 0x7F
    v2v = 0x0F
    important2 = 0
    v2Class = 0
  struct.pack_into('BBBBBBBB', msg, 0,lane + (v1Class << 3) + (important1 << 7),v1x, v1v + ((v1y << 4) & 0xF0),
    int((v1y >> 4) & 0x07) + ((v1Id << 3) & 0xF8), int((v1Id >> 5) & 0x03) + (v2Class << 2) +(important2 << 6) + ((v2x << 7) & 0x80),
    int((v2x >> 1) & 0x7F) + ((v2v << 7) & 0x80), int((v2v >> 1) & 0x07) + ((v2y << 3) & 0xF8),
    int((v2y >> 5) & 0x03) + ((v2Id << 2) & 0xFC))
  return [msg_id, 0, msg.raw, 0]

def create_fake_DAS_msg2(hiLoBeamStatus,hiLoBeamReason,ahbIsEnabled,fleet_speed_state):
  msg_id = 0x65A
  msg_len = 3
  msg = create_string_buffer(msg_len)
  struct.pack_into('BBB', msg, 0, hiLoBeamStatus, hiLoBeamReason,(1 if ahbIsEnabled else 0) + (fleet_speed_state << 1))
  return [msg_id, 0, msg.raw, 0]


def create_fake_DAS_msg(speed_control_enabled,speed_override,apUnavailable, collision_warning, op_status, \
                 acc_speed_kph, \
                 turn_signal_needed,forward_collission_warning,adaptive_cruise, hands_on_state, \
                 cc_state, pcc_available, alca_state, \
                 acc_speed_limit, # IC cruise speed, kph or mph
                 legal_speed_limit,
                 apply_angle,
                 enable_steer_control, 
                 park_brake_request):
  msg_id = 0x659 #we will use DAS_udsRequest to send this info to IC
  msg_len = 8
  msg = create_string_buffer(msg_len)
  units_included = 1
  c_apply_steer = int(((int( apply_angle * 10 + 0x4000 )) & 0x7FFF) + (enable_steer_control << 15))
  struct.pack_into('BBBBBBBB', msg, 0,int((speed_control_enabled << 7) + (speed_override << 6) + (apUnavailable << 5) + (collision_warning << 4) + op_status), \
      int(acc_speed_kph), \
      int((turn_signal_needed << 6) + (units_included << 5) + (forward_collission_warning << 4)  + (adaptive_cruise << 3) + hands_on_state), \
      int((cc_state << 6) + (pcc_available << 5) + alca_state), \
      int(acc_speed_limit + 0.5), # IC rounds current speed, so we need to round cruise speed the same way
      int((legal_speed_limit & 0x1F) + ((park_brake_request << 5) & 0x20)), #positions 7 and 6 not used yet
      int(c_apply_steer & 0xFF),
      int((c_apply_steer >> 8) & 0xFF))
  return [msg_id, 0, msg.raw, 0]

def create_fake_DAS_obj_lane_msg(leadDx,leadDy,leadClass,rLine,lLine,curv0,curv1,curv2,curv3,laneRange,laneWidth):
  msg_id = 0x557
  msg_len = 8
  f = IC_LANE_SCALE
  f2 = f * f
  f3 = f2 * f
  if (leadDx > 127):
    leadDx = 127
  if (leadDx < 0 ):
    leadDx = 0
  if (leadDy > 22):
    leadDy = 22
  if (leadDy < -22 ):
    leadDy = -22
  tLeadDx = int(leadDx / 0.5)
  tLeadDy = int((22.5 + leadDy) / 0.35)
  tCurv0 = (int((curv0 + 3.5)/0.035)) & 0xFF
  tCurv1 = (int((clip(curv1*f,-0.2,0.2) + 0.2)/0.0016)) & 0xFF
  tCurv2 = (int((clip(curv2*f2,-0.0025,0.0025)  + 0.0025)/0.00002)) & 0xFF
  tCurv3 = (int((clip(curv3*f3,-0.00003,0.00003) + 0.00003)/0.00000024)) & 0xFF
  lWidth = (int((laneWidth - 2.0)/0.3125)) & 0x0F
  msg = create_string_buffer(msg_len)
  struct.pack_into('BBBBBBBB',msg ,0 , tLeadDx,tLeadDy,(lWidth << 4) + (lLine << 2) + rLine, tCurv0,tCurv1,tCurv2,tCurv3,((leadClass & 0x03) << 6) + int(laneRange/4))
  return [msg_id,0,msg.raw,0]

def create_fake_DAS_sign_msg(roadSignType,roadSignStopDist,roadSignColor,roadSignControlActive):
  msg_id = 0x556
  msg_len = 4
  orientation = 0x00 #unknown
  arrow = 0x04 #unknown
  source = 0x02 #vision
  roadSignStopDist_t = (((roadSignStopDist + 20) / 0.2) & 0x3FF)
  sign1 = ((roadSignType & 0x03) << 6) +(roadSignColor << 3) + 0x04
  sign2 = ((roadSignStopDist_t & 0x03) << 6) + int((roadSignType >> 2) & 0xFF)
  sign3 = int(roadSignStopDist_t >> 2)
  sign4 = (orientation << 6) + (arrow << 3) + (source << 1) + roadSignControlActive
  msg = create_string_buffer(msg_len)
  struct.pack_into('BBBB',msg ,0 , sign1,sign2,sign3,sign4)
  return [msg_id,0,msg.raw,0]

def create_fake_DAS_warning(DAS_211_accNoSeatBelt, DAS_canErrors, \
            DAS_202_noisyEnvironment, DAS_doorOpen, DAS_notInDrive, enableDasEmulation, enableRadarEmulation, \
            stopSignWarning, stopLightWarning, \
            DAS_222_accCameraBlind, DAS_219_lcTempUnavailableSpeed, DAS_220_lcTempUnavailableRoad, DAS_221_lcAborting, \
            DAS_207_lkasUnavailable,DAS_208_rackDetected, DAS_025_steeringOverride, ldwStatus,FLAG_notUsed,useWithoutHarness):
  msg_id = 0x554
  msg_len = 3
  fd = 0
  rd = 0
  if enableDasEmulation:
    fd = 1
  if enableRadarEmulation:
    rd = 1
  utr = 0
  wh = 0
  if useWithoutHarness:
    wh = 1
  warn1 = (stopLightWarning<< 7) + (rd << 6) + (fd << 5) + (DAS_211_accNoSeatBelt << 4) + (DAS_canErrors << 3) + (DAS_202_noisyEnvironment << 2) + (DAS_doorOpen << 1) + DAS_notInDrive
  warn2 = stopSignWarning + (DAS_222_accCameraBlind << 1) + (DAS_219_lcTempUnavailableSpeed << 2) + (DAS_220_lcTempUnavailableRoad << 3) + (DAS_221_lcAborting << 4) + (DAS_207_lkasUnavailable << 5) + (DAS_208_rackDetected << 6) + (DAS_025_steeringOverride << 7)
  warn3 = ldwStatus + (FLAG_notUsed << 3) + (wh << 4)
  msg = create_string_buffer(msg_len)
  struct.pack_into('BBB',msg ,0 , warn1,warn2,warn3)
  return [msg_id,0,msg.raw,0]


def create_cruise_adjust_msg(spdCtrlLvr_stat, turnIndLvr_Stat, real_steering_wheel_stalk):
  """Creates a CAN message from the cruise control stalk.
  Simluates pressing the cruise control stalk (STW_ACTN_RQ.SpdCtrlLvr_Stat) 
  and turn signal stalk (STW_ACTN_RQ.TurnIndLvr_Stat)
  It is probably best not to flood these messages so that the real
  stalk works normally.
  Args:
    spdCtrlLvr_stat: Int value of dbc entry STW_ACTN_RQ.SpdCtrlLvr_Stat
      (allowing us to simulate pressing the cruise stalk up or down)
      None means no change
    TurnIndLvr_Stat: Int value of dbc entry STW_ACTN_RQ.TurnIndLvr_Stat
      (allowing us to simulate pressing the turn signal up or down)
      None means no change
    real_steering_wheel_stalk: Previous STW_ACTN_RQ message sent by the real
      stalk. When sending these artifical messages for cruise control, we want
      to mimic whatever windshield wiper and highbeam settings the car is
      currently sending.
    
  """
  msg_id = 0x045  # 69 in hex, STW_ACTN_RQ
  msg_len = 8
  msg = create_string_buffer(msg_len)
  # Do not send messages that conflict with the driver's actual actions on the
  # steering wheel stalk. To ensure this, copy all the fields you can from the
  # real cruise stalk message.
  fake_stalk = real_steering_wheel_stalk.copy()

  if spdCtrlLvr_stat is not None:
    # if accelerating, override VSL_Enbl_Rq to 1.
    if spdCtrlLvr_stat in [4, 16]:
      fake_stalk['VSL_Enbl_Rq'] = 1
    fake_stalk['SpdCtrlLvr_Stat'] = spdCtrlLvr_stat
  if turnIndLvr_Stat is not None:
    fake_stalk['TurnIndLvr_Stat'] = turnIndLvr_Stat
  # message count should be 1 more than the previous (and loop after 16)
  fake_stalk['MC_STW_ACTN_RQ'] = (int(round(fake_stalk['MC_STW_ACTN_RQ'])) + 1) % 16
  # CRC should initially be 0 before a new one is calculated.
  fake_stalk['CRC_STW_ACTN_RQ'] = 0
  
  # Set the first byte, containing cruise control
  struct.pack_into('B', msg, 0,
                   (fake_stalk['SpdCtrlLvr_Stat']) +
                   (int(round(fake_stalk['VSL_Enbl_Rq'])) << 6))
  # Set the 2nd byte, containing DTR_Dist_Rq
  struct.pack_into('B', msg, 1,  int(fake_stalk['DTR_Dist_Rq']))
  # Set the 3rd byte, containing turn indicator, highbeams, and wiper wash
  struct.pack_into('B', msg, 2,
                   int(round(fake_stalk['TurnIndLvr_Stat'])) +
                   (int(round(fake_stalk['HiBmLvr_Stat'])) << 2) +
                   (int(round(fake_stalk['WprWashSw_Psd'])) << 4) +
                   (int(round(fake_stalk['WprWash_R_Sw_Posn_V2'])) << 6)
                  )
  # Set the 7th byte, containing the wipers and message counter.
  struct.pack_into('B', msg, 6,
                   int(round(fake_stalk['WprSw6Posn'])) +
                   (fake_stalk['MC_STW_ACTN_RQ'] << 4))
  
  # Finally, set the CRC for the message. Must be calculated last!
  fake_stalk['CRC_STW_ACTN_RQ'] = add_tesla_crc(msg=msg, msg_len=7)
  struct.pack_into('B', msg, msg_len-1, int(fake_stalk['CRC_STW_ACTN_RQ']))

  return [msg_id, 0, msg.raw, 0]

