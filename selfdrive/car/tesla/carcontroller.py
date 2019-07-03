import datetime
from cereal import log,ui
from common.params import Params
from collections import namedtuple
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from selfdrive.car.tesla import teslacan
from selfdrive.car.tesla.values import AH, CM
from selfdrive.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from selfdrive.car.modules.ALCA_module import ALCAController
from selfdrive.car.modules.GYRO_module import GYROController
from selfdrive.car.tesla.ACC_module import ACCController
from selfdrive.car.tesla.PCC_module import PCCController
from selfdrive.car.tesla.HSO_module import HSOController
from selfdrive.car.tesla.movingaverage import MovingAverage
import zmq
import selfdrive.messaging as messaging
from selfdrive.services import service_list

# Steer angle limits
ANGLE_MAX_BP = [0., 27., 36.]
ANGLE_MAX_V = [410., 92., 36.]

ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .25]     # windup limit
ANGLE_DELTA_VU = [5., 3.5, 0.8]   # unwind limit
#steering adjustment with speed
DES_ANGLE_ADJUST_FACTOR_BP = [0.,13., 44.]
DES_ANGLE_ADJUST_FACTOR = [1.0, 1.0, 1.0]

#LDW WARNING LEVELS
LDW_WARNING_1 = 1.0
LDW_WARNING_2 = 0.9
LDW_LANE_PROBAB = 0.3

def gen_solution(CS):
  fix = 0
  if CS.gpsAccuracy < 2:
    fix = 1
  timestamp = int(((datetime.datetime.now() - datetime.datetime(1970,1,1)).total_seconds())*1e+03)
  gps_fix = {'bearing': CS.gpsHeading,  # heading of motion in degrees
             'altitude': CS.gpsElevation,  # altitude above ellipsoid
             'latitude': CS.gpsLatitude,  # latitude in degrees
             'longitude': CS.gpsLongitude,  # longitude in degrees
             'speed': CS.gpsVehicleSpeed,  # ground speed in meters
             'accuracy': CS.gpsAccuracy,  # horizontal accuracy (1 sigma?)
             'timestamp': timestamp,  # UTC time in ms since start of UTC stime
             'vNED': [0.,0.,0.],  # velocity in NED frame in m/s
             'speedAccuracy': 0.,  # speed accuracy in m/s
             'verticalAccuracy': 0.,  # vertical accuracy in meters
             'bearingAccuracy': 0.,  # heading accuracy in degrees
             'source': 'ublox',
             'flags': fix, # 1 of gpsAccuracy less than 2 meters
  }
  return log.Event.new_message(gpsLocationExternal=gps_fix)

def process_hud_alert(hud_alert):
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0
  if hud_alert == AH.NONE:          # no alert
    pass
  elif hud_alert == AH.FCW:         # FCW
    fcw_display = hud_alert[1]
  elif hud_alert == AH.STEER:       # STEER
    steer_required = hud_alert[1]
  else:                             # any other ACC alert
    acc_alert = hud_alert[1]

  return fcw_display, steer_required, acc_alert


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "mini_car", "car", "X4",
                      "lanes", "beep", "chime", "fcw", "acc_alert", "steer_required"])



class CarController(object):
  def __init__(self, dbc_name):
    self.params = Params()
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.packer = CANPacker(dbc_name)
    self.epas_disabled = True
    self.last_angle = 0.
    self.last_accel = 0.
    self.ALCA = ALCAController(self,True,True)  # Enabled and SteerByAngle both True
    self.ACC = ACCController(self)
    self.PCC = PCCController(self)
    self.HSO = HSOController(self)
    self.GYRO = GYROController()
    self.sent_DAS_bootID = False
    self.context = zmq.Context()
    self.poller = zmq.Poller()
    self.speedlimit = None
    self.trafficevents = messaging.sub_sock(self.context, service_list['trafficEvents'].port, conflate=True, poller=self.poller)
    self.pathPlan = messaging.sub_sock(self.context, service_list['pathPlan'].port, conflate=True, poller=self.poller)
    self.radarState = messaging.sub_sock(self.context, service_list['radarState'].port, conflate=True, poller=self.poller)
    self.icCarLR = messaging.sub_sock(self.context, service_list['uiIcCarLR'].port, conflate=True, poller=self.poller)
    self.gpsLocationExternal = None 
    self.speedlimit_ms = 0.
    self.speedlimit_valid = False
    self.speedlimit_units = 0
    self.opState = 0 # 0-disabled, 1-enabled, 2-disabling, 3-unavailable, 5-warning
    self.accPitch = 0.
    self.accRoll = 0.
    self.accYaw = 0.
    self.magPitch = 0.
    self.magRoll = 0.
    self.magYaw = 0.
    self.gyroPitch = 0.
    self.gyroRoll = 0.
    self.gyroYaw = 0.
    self.set_speed_limit_active = False
    self.speed_limit_offset = 0.
    self.speed_limit_for_cc = 0.

    # for warnings
    self.warningCounter = 0
    self.DAS_206_apUnavailable = 0
    self.DAS_222_accCameraBlind = 0 #DAS_206 lkas not ebabled
    self.DAS_219_lcTempUnavailableSpeed = 0
    self.DAS_220_lcTempUnavailableRoad = 0
    self.DAS_221_lcAborting = 0
    self.DAS_211_accNoSeatBelt = 0
    self.DAS_207_lkasUnavailable = 0 #use for manual steer?
    self.DAS_208_rackDetected = 0 #use for low battery?
    self.DAS_202_noisyEnvironment = 0 #use for planner error?
    self.DAS_025_steeringOverride = 0 #another one to use for manual steer?
    self.warningNeeded = 0

    # items for IC integration for Lane and Lead Car
    self.average_over_x_pathplan_values = 1 
    self.curv0Matrix =  MovingAverage(self.average_over_x_pathplan_values)
    self.curv1Matrix =  MovingAverage(self.average_over_x_pathplan_values) 
    self.curv2Matrix =  MovingAverage(self.average_over_x_pathplan_values) 
    self.curv3Matrix =  MovingAverage(self.average_over_x_pathplan_values)
    self.leadDxMatrix = MovingAverage(self.average_over_x_pathplan_values)
    self.leadDyMatrix = MovingAverage(self.average_over_x_pathplan_values)
    self.leadDx = 0.
    self.leadDy = 0.
    self.leadClass = 0
    self.leadVx = 0.
    self.leadId = 0
    self.lead2Dx = 0.
    self.lead2Dy = 0.
    self.lead2Class = 0
    self.lead2Vx = 0.
    self.lead2Id = 0
    self.lLine = 0
    self.rLine = 0
    self.curv0 = 0. 
    self.curv1 = 0. 
    self.curv2 = 0. 
    self.curv3 = 0. 
    self.visionCurvC0 = 0.
    self.laneRange = 50  #max is 160m but OP has issues with precision beyond 50
    self.useZeroC0 = False
    self.useMap = True
    self.clipC0 = False
    self.useMapOnly = True
    self.laneWidth = 0.

    self.stopSign_visible = False
    self.stopSign_distance = 1000.
    self.stopSign_action = 0
    self.stopSign_resume = False

    self.stopLight_visible = False
    self.stopLight_distance = 1000.
    self.stopLight_action = 0
    self.stopLight_resume = False
    self.stopLight_color = 0. #0-unknown, 1-red, 2-yellow, 3-green

    self.stopSignWarning = 0
    self.stopLightWarning = 0
    self.stopSignWarning_last = 0
    self.stopLightWarning_last = 0
    self.roadSignType = 0xFF
    self.roadSignStopDist = 1000.
    self.roadSignColor = 0.
    self.roadSignControlActive = 0
    self.roadSignType_last = 0xFF

    self.roadSignDistanceWarning = 50.

    self.alca_enabled = False
    self.ldwStatus = 0
    self.prev_ldwStatus = 0

    self.radarVin_idx = 0

  def reset_traffic_events(self):
    self.stopSign_visible = False
    self.stopSign_distance = 1000.
    self.stopSign_action = 0
    self.stopSign_resume = False

    self.stopLight_visible = False
    self.stopLight_distance = 1000.
    self.stopLight_action = 0
    self.stopLight_resume = False
    self.stopLight_color = 0. #0-unknown, 1-red, 2-yellow, 3-green

  def checkWhichSign(self):
    self.stopSignWarning = 0
    self.stopLightWarning = 0
    self.roadSignType_last = self.roadSignType
    self.roadSignType = 0xFF
    self.roadSignStopDist = 1000.
    self.roadSignColor = 0
    self.roadSignControlActive = 0
    if (self.stopSign_distance < self.stopLight_distance):
      self.roadSignType = 0x00
      self.roadSignStopDist = self.stopSign_distance
      self.roadSignColor = 0
      self.roadSignControlActive = self.stopSign_resume
      if (self.stopSign_distance < self.roadSignDistanceWarning ):
        self.stopSignWarning = 1
    elif (self.stopLight_distance < self.stopSign_distance ):
      self.roadSignType = 0x01
      self.roadSignStopDist = self.stopLight_distance
      self.roadSignColor = self.stopLight_color
      self.roadSignControlActive = self.stopLight_resume
      if (self.stopLight_distance < self.roadSignDistanceWarning ) and (self.roadSignColor == 1):
        self.stopLightWarning = 1
    



  def update(self, enabled, CS, frame, actuators, \
             pcm_speed, pcm_override, pcm_cancel_cmd, pcm_accel, \
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert, \
             snd_beep, snd_chime,leftLaneVisible,rightLaneVisible):

    if (not enabled) and (self.ALCA.laneChange_cancelled):
      self.ALCA.laneChange_cancelled = False
      self.ALCA.laneChange_cancelled_counter = 0
      self.warningNeeded = 1
    if self.warningCounter > 0:
      self.warningCounter = self.warningCounter - 1
      if self.warningCounter == 0:
        self.warningNeeded = 1
    if self.warningCounter == 0 or not enabled:
        # when zero reset all warnings
        self.DAS_222_accCameraBlind = 0 #we will see what we can use this for
        self.DAS_219_lcTempUnavailableSpeed = 0
        self.DAS_220_lcTempUnavailableRoad = 0
        self.DAS_221_lcAborting = 0
        self.DAS_211_accNoSeatBelt = 0
        self.DAS_207_lkasUnavailable = 0 #use for manual not in drive?
        self.DAS_208_rackDetected = 0 #use for low battery?
        self.DAS_202_noisyEnvironment = 0 #use for planner error?
        self.DAS_025_steeringOverride = 0 #use for manual steer?
        self.DAS_206_apUnavailable = 0 #Ap disabled from CID

    if CS.keepEonOff:
      if CS.cstm_btns.get_button_status("dsp") != 9:
        CS.cstm_btns.set_button_status("dsp",9)
    else:
      if CS.cstm_btns.get_button_status("dsp") != 1:
        CS.cstm_btns.set_button_status("dsp",1) 
    # """ Controls thread """

    if not CS.useTeslaMapData:
      if self.speedlimit is None:
        self.speedlimit = messaging.sub_sock(self.context, service_list['liveMapData'].port, conflate=True, poller=self.poller)


    # *** no output if not enabled ***
    if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = True

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    if hud_show_lanes:
      hud_lanes = 1
    else:
      hud_lanes = 0

    # TODO: factor this out better
    if enabled:
      if hud_show_car:
        hud_car = 2
      else:
        hud_car = 1
    else:
      hud_car = 0
    
    # For lateral control-only, send chimes as a beep since we don't send 0x1fa
    #if CS.CP.radarOffCan:

    #print chime, alert_id, hud_alert
    fcw_display, steer_required, acc_alert = process_hud_alert(hud_alert)

    hud = HUDData(int(pcm_accel), int(round(hud_v_cruise)), 1, hud_car,
                  0xc1, hud_lanes, int(snd_beep), snd_chime, fcw_display, acc_alert, steer_required)
 
    if not all(isinstance(x, int) and 0 <= x < 256 for x in hud):
      print "INVALID HUD", hud
      hud = HUDData(0xc6, 255, 64, 0xc0, 209, 0x40, 0, 0, 0, 0)

    # **** process the car messages ****

    # *** compute control surfaces ***

    STEER_MAX = 420
    # Prevent steering while stopped
    MIN_STEERING_VEHICLE_VELOCITY = 0.05 # m/s
    vehicle_moving = (CS.v_ego >= MIN_STEERING_VEHICLE_VELOCITY)
    
    # Basic highway lane change logic
    changing_lanes = CS.right_blinker_on or CS.left_blinker_on

    #upodate custom UI buttons and alerts
    CS.UE.update_custom_ui()
      
    if (frame % 100 == 0):
      CS.cstm_btns.send_button_info()
      #read speed limit params
      if CS.hasTeslaIcIntegration:
        self.set_speed_limit_active = True
        self.speed_limit_offset = CS.userSpeedLimitOffsetKph
        self.speed_limit_for_cc = CS.userSpeedLimitKph
        #print self.speed_limit_for_cc
      else:
        self.set_speed_limit_active = (self.params.get("SpeedLimitOffset") is not None) and (self.params.get("LimitSetSpeed") == "1")
        if self.set_speed_limit_active:
          self.speed_limit_offset = float(self.params.get("SpeedLimitOffset"))
        else:
          self.speed_limit_offset = 0.
        if (self.params.get("IsMetric") == "0"):
          self.speed_limit_offset = self.speed_limit_offset * CV.MPH_TO_MS
    if CS.useTeslaGPS:
      if self.gpsLocationExternal is None:
        self.gpsLocationExternal = messaging.pub_sock(self.context, service_list['gpsLocationExternal'].port)
      sol = gen_solution(CS)
      sol.logMonoTime = int(sec_since_boot() * 1e9)
      self.gpsLocationExternal.send(sol.to_bytes())

    #get pitch/roll/yaw every 0.1 sec
    if (frame %10 == 0):
      (self.accPitch, self.accRoll, self.accYaw),(self.magPitch, self.magRoll, self.magYaw),(self.gyroPitch, self.gyroRoll, self.gyroYaw) = self.GYRO.update(CS.v_ego,CS.a_ego,CS.angle_steers)
      CS.UE.uiGyroInfoEvent(self.accPitch, self.accRoll, self.accYaw,self.magPitch, self.magRoll, self.magYaw,self.gyroPitch, self.gyroRoll, self.gyroYaw)

    # Update statuses for custom buttons every 0.1 sec.
    if self.ALCA.pid is None:
      self.ALCA.set_pid(CS)
    if (frame % 10 == 0):
      self.ALCA.update_status((CS.cstm_btns.get_button_status("alca") > 0) and ((CS.enableALCA and not CS.hasTeslaIcIntegration) or (CS.hasTeslaIcIntegration and CS.alcaEnabled)))
      #print CS.cstm_btns.get_button_status("alca")
    
    pedal_can_sends = []
    
    if CS.pedal_interceptor_available:
      #update PCC module info
      pedal_can_sends = self.PCC.update_stat(CS, True)
      self.ACC.enable_adaptive_cruise = False
    else:
      # Update ACC module info.
      self.ACC.update_stat(CS, True)
      self.PCC.enable_pedal_cruise = False
    
    # Update HSO module info.
    human_control = False

    # update CS.v_cruise_pcm based on module selected.
    if self.ACC.enable_adaptive_cruise:
      CS.v_cruise_pcm = self.ACC.acc_speed_kph
    elif self.PCC.enable_pedal_cruise:
      CS.v_cruise_pcm = self.PCC.pedal_speed_kph
    else:
      CS.v_cruise_pcm = CS.v_cruise_actual
    # Get the angle from ALCA.
    turn_signal_needed = 0
    alca_steer = 0.
    apply_angle, alca_steer,self.alca_enabled, turn_signal_needed = self.ALCA.update(enabled, CS, frame, actuators,self.visionCurvC0)
    apply_angle = -apply_angle  # Tesla is reversed vs OP.
    human_control = self.HSO.update_stat(self,CS, enabled, actuators, frame)
    human_lane_changing = changing_lanes and not self.alca_enabled
    enable_steer_control = (enabled
                            and not human_lane_changing
                            and not human_control 
                            and  vehicle_moving)
    
    angle_lim = interp(CS.v_ego, ANGLE_MAX_BP, ANGLE_MAX_V)
    apply_angle = clip(apply_angle, -angle_lim, angle_lim)
    # Windup slower.
    if self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle):
      angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_V)
    else:
      angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_VU)

    des_angle_factor = interp(CS.v_ego, DES_ANGLE_ADJUST_FACTOR_BP, DES_ANGLE_ADJUST_FACTOR )
    if self.alca_enabled or not CS.enableSpeedVariableDesAngle:
      des_angle_factor = 1.
    #BB disable limits to test 0.5.8
    # apply_angle = clip(apply_angle * des_angle_factor, self.last_angle - angle_rate_lim, self.last_angle + angle_rate_lim) 
    # If human control, send the steering angle as read at steering wheel.
    if human_control:
      apply_angle = CS.angle_steers

    # Send CAN commands.
    can_sends = []

    #if using radar, we need to send the VIN
    if CS.useTeslaRadar and (frame % 100 == 0):
      useRadar=0
      if CS.useTeslaRadar:
        useRadar=1
      can_sends.append(teslacan.create_radar_VIN_msg(self.radarVin_idx,CS.radarVIN,1,0x108,useRadar,CS.radarPosition,CS.radarEpasType))
      self.radarVin_idx += 1
      self.radarVin_idx = self.radarVin_idx  % 3

    #First we emulate DAS.
    # DAS_longC_enabled (1),DAS_speed_override (1),DAS_apUnavailable (1), DAS_collision_warning (1),  DAS_op_status (4)
    # DAS_speed_kph(8), 
    # DAS_turn_signal_request (2),DAS_forward_collision_warning (2), DAS_hands_on_state (4), 
    # DAS_cc_state (2), DAS_usingPedal(1),DAS_alca_state (5),
    # DAS_acc_speed_limit_mph (8), 
    # DAS_speed_limit_units(8)
    #send fake_das data as 0x553
    # TODO: forward collission warning

    if CS.hasTeslaIcIntegration:
        self.set_speed_limit_active = True
        self.speed_limit_offset = CS.userSpeedLimitOffsetKph
        # only change the speed limit when we have a valid vaue
        if CS.userSpeedLimitKph >= 10:
          self.speed_limit_for_cc = CS.userSpeedLimitKph

    if CS.useTeslaMapData:    
      self.speedlimit_ms = CS.speedLimitKph * CV.KPH_TO_MS
      self.speedlimit_valid = True
      if self.speedlimit_ms == 0:
        self.speedlimit_valid = False
      if (self.params.get("IsMetric") == "1"):
        self.speedlimit_units = self.speedlimit_ms * CV.MS_TO_KPH + 0.5
      else:
        self.speedlimit_units = self.speedlimit_ms * CV.MS_TO_MPH + 0.5
    if frame % 10 == 0:
      #get speed limit
      for socket, _ in self.poller.poll(1):
        if not CS.useTeslaMapData:
          if socket is self.speedlimit:
            lmd = messaging.recv_one(socket).liveMapData
            self.speedlimit_ms = lmd.speedLimit
            self.speedlimit_valid = lmd.speedLimitValid
            if (self.params.get("IsMetric") == "1"):
              self.speedlimit_units = self.speedlimit_ms * CV.MS_TO_KPH + 0.5
            else:
              self.speedlimit_units = self.speedlimit_ms * CV.MS_TO_MPH + 0.5
            self.speed_limit_for_cc = self.speedlimit_ms * CV.MS_TO_KPH
        #to show lead car on IC
        if socket is self.radarState:
          leads = messaging.recv_one(socket).radarState
          if leads is not None:
            lead_1 = leads.leadOne
            lead_2 = leads.leadTwo
            if (lead_1 is not None) and lead_1.status:
              self.leadDx = lead_1.dRel
              self.leadDy = self.curv0-lead_1.yRel
              self.leadId = lead_1.trackId
              self.leadClass = lead_1.oClass 
              self.leadVx = lead_1.vRel
              if (self.leadId <= 0) or (self.leadId == 63):
                self.leadId = 61
            else:
              self.leadDx = 0.
              self.leadDy = 0.
              self.leadClass = 0
              self.leadId = 0
              self.leadVx = 0xF
            if (lead_2 is not None) and lead_2.status:
              self.lead2Dx = lead_2.dRel
              self.lead2Dy = self.curv0-lead_2.yRel
              self.lead2Id = lead_2.trackId
              self.lead2Class = lead_2.oClass 
              self.lead2Vx = lead_2.vRel
              if (self.lead2Id <= 0) or (self.lead2Id == 63):
                self.leadId = 62
            else:
              self.lead2Dx = 0.
              self.lead2Dy = 0.
              self.lead2Class = 0
              self.lead2Id = 0
              self.lead2Vx = 0xF
            can_sends.append(teslacan.create_DAS_LR_object_msg(0,self.leadClass, self.leadId,
                  self.leadDx,self.leadDy,self.leadVx,self.lead2Class,
                  self.lead2Id,self.lead2Dx,self.lead2Dy,self.lead2Vx))
        #to show curvature and lanes on IC
        if socket is self.pathPlan:
          pp = messaging.recv_one(socket).pathPlan
          if pp.valid:
            if pp.lProb > 0.75:
              self.lLine = 3
            elif pp.lProb > 0.5:
              self.lLine = 2
            elif pp.lProb > 0.25:
              self.lLine = 1
            else:
              self.lLine = 0
            if pp.rProb > 0.75:
              self.rLine = 3
            elif pp.rProb > 0.5:
              self.rLine = 2
            elif pp.rProb > 0.25:
              self.rLine = 1
            else:
              self.rLine = 0
            #first we clip to the AP limits of the coefficients
            self.curv0 = -clip(pp.cPoly[3],-3.5,3.5) #self.curv0Matrix.add(-clip(pp.cPoly[3],-3.5,3.5))
            self.curv1 = -clip(pp.cPoly[2],-0.2,0.2) #self.curv1Matrix.add(-clip(pp.cPoly[2],-0.2,0.2))
            self.curv2 = -clip(pp.cPoly[1],-0.0025,0.0025) #self.curv2Matrix.add(-clip(pp.cPoly[1],-0.0025,0.0025))
            self.curv3 = -clip(pp.cPoly[0],-0.00003,0.00003) #self.curv3Matrix.add(-clip(pp.cPoly[0],-0.00003,0.00003))
            self.laneWidth = pp.laneWidth
            self.laneRange = 50 # it is fixed in OP at 50m pp.viewRange
            self.visionCurvC0 = self.curv0
            self.prev_ldwStatus = self.ldwStatus
            self.ldwStatus = 0
            if self.alca_enabled:
              #exagerate position a little during ALCA to make lane change look smoother on IC
              if self.ALCA.laneChange_over_the_line:
                self.curv0 = self.ALCA.laneChange_direction * self.laneWidth - self.curv0
              self.curv0 = clip(self.curv0, -3.5, 3.5)
            else:
              if CS.enableLdw and (not CS.blinker_on) and (CS.v_ego > 15.6) and (not CS.steer_override):
                if pp.lProb > LDW_LANE_PROBAB:
                  lLaneC0 = -pp.lPoly[3]
                  if abs(lLaneC0) < LDW_WARNING_2:
                    self.ldwStatus = 3
                  elif  abs(lLaneC0) < LDW_WARNING_1:
                    self.ldwStatus = 1
                if pp.rProb > LDW_LANE_PROBAB:
                  rLaneC0 = -pp.rPoly[3]
                  if abs(rLaneC0) < LDW_WARNING_2:
                    self.ldwStatus = 3
                  elif  abs(rLaneC0) < LDW_WARNING_1:
                    self.ldwStatus = 1
            if not(self.prev_ldwStatus == self.ldwStatus):
              self.warningNeeded = 1
              if self.ldwStatus > 0:
                self.warningCounter = 50
          else:
            self.lLine = 0
            self.rLine = 0
            self.curv0 = self.curv0Matrix.add(0.)
            self.curv1 = self.curv1Matrix.add(0.)
            self.curv2 = self.curv2Matrix.add(0.)
            self.curv3 = self.curv3Matrix.add(0.)
        if socket is self.icCarLR:
          icCarLR_msg = ui.ICCarsLR.from_bytes(socket.recv())
          if icCarLR_msg is not None:
            #for icCarLR_msg in icCarLR_list:
            can_sends.append(teslacan.create_DAS_LR_object_msg(1,icCarLR_msg.v1Type,icCarLR_msg.v1Id,
                icCarLR_msg.v1Dx,icCarLR_msg.v1Dy,icCarLR_msg.v1Vrel,icCarLR_msg.v2Type,
                icCarLR_msg.v2Id,icCarLR_msg.v2Dx,icCarLR_msg.v2Dy,icCarLR_msg.v2Vrel))
            can_sends.append(teslacan.create_DAS_LR_object_msg(2,icCarLR_msg.v3Type,icCarLR_msg.v3Id,
                icCarLR_msg.v3Dx,icCarLR_msg.v3Dy,icCarLR_msg.v3Vrel,icCarLR_msg.v4Type,
                icCarLR_msg.v4Id,icCarLR_msg.v4Dx,icCarLR_msg.v4Dy,icCarLR_msg.v4Vrel))
        if socket is self.trafficevents:
          self.reset_traffic_events()
          tr_ev_list = messaging.recv_sock(socket)
          if tr_ev_list is not None:
            for tr_ev in tr_ev_list.trafficEvents:
              if tr_ev.type == 0x00:
                if (tr_ev.distance < self.stopSign_distance):
                  self.stopSign_visible = True
                  self.stopSign_distance = tr_ev.distance 
                  self.stopSign_action = tr_ev.action
                  self.stopSign_resume = tr_ev.resuming
              if tr_ev.type ==  0x04:
                if (tr_ev.distance < self.stopLight_distance):
                  self.stopLight_visible = True
                  self.stopLight_distance = tr_ev.distance
                  self.stopLight_action = tr_ev.action
                  self.stopLight_resume = tr_ev.resuming
                  self.stopLight_color = 1. #0-unknown, 1-red, 2-yellow, 3-green
              if tr_ev.type == 0x01:
                if (tr_ev.distance < self.stopLight_distance):
                  self.stopLight_visible = True
                  self.stopLight_distance = tr_ev.distance
                  self.stopLight_action = tr_ev.action
                  self.stopLight_resume = tr_ev.resuming
                  self.stopLight_color = 1. #0-unknown, 1-red, 2-yellow, 3-green
              if tr_ev.type == 0x02:
                if (tr_ev.distance < self.stopLight_distance):
                  self.stopLight_visible = True
                  self.stopLight_distance = tr_ev.distance
                  self.stopLight_action = tr_ev.action
                  self.stopLight_resume = tr_ev.resuming
                  self.stopLight_color = 2. #0-unknown, 1-red, 2-yellow, 3-green
              if tr_ev.type == 0x03:
                if (tr_ev.distance < self.stopLight_distance):
                  self.stopLight_visible = True
                  self.stopLight_distance = tr_ev.distance
                  self.stopLight_action = tr_ev.action
                  self.stopLight_resume = tr_ev.resuming
                  self.stopLight_color = 3. #0-unknown, 1-red, 2-yellow, 3-green
            self.checkWhichSign()
            if not ((self.roadSignType_last == self.roadSignType) and (self.roadSignType == 0xFF)):
               can_sends.append(teslacan.create_fake_DAS_sign_msg(self.roadSignType,self.roadSignStopDist,self.roadSignColor,self.roadSignControlActive))
    if (CS.roadCurvRange > 20) and self.useMap:
      if self.useZeroC0:
        self.curv0 = 0.
      elif self.clipC0:
        self.curv0 = -clip(CS.roadCurvC0,-0.5,0.5)
      #else:
      #  self.curv0 = -CS.roadCurvC0
      #if CS.v_ego > 9:
      #  self.curv1 = -CS.roadCurvC1
      #else:
      #  self.curv1 = 0.
      self.curv2 = -CS.roadCurvC2
      self.curv3 = -CS.roadCurvC3
      self.laneRange = CS.roadCurvRange
    #else:
    #  self.curv0 = 0.
    #  self.curv1 = 0.
    #  self.curv2 = 0.
    #  self.curv3 = 0.
    #  self.laneRange = 0
    
    if (CS.csaRoadCurvRange > 2.) and self.useMap and not self.useMapOnly:
      self.curv2 = -CS.csaRoadCurvC2
      self.curv3 = -CS.csaRoadCurvC3
      #if self.laneRange > 0:
      #  self.laneRange = min(self.laneRange,CS.csaRoadCurvRange)
      #else:
      self.laneRange = CS.csaRoadCurvRange
    elif (CS.csaOfframpCurvRange > 2.) and self.useMap and not self.useMapOnly:
      #self.curv2 = -CS.csaOfframpCurvC2
      #self.curv3 = -CS.csaOfframpCurvC3
      #self.curv0 = 0.
      #self.curv1 = 0.
      #if self.laneRange > 0:
      #  self.laneRange = min(self.laneRange,CS.csaOfframpCurvRange)
      #else:
      self.laneRange = CS.csaOfframpCurvRange
    else:
      self.laneRange = 50
    self.laneRange = int(clip(self.laneRange,0,159))
    op_status = 0x02
    hands_on_state = 0x00
    forward_collision_warning = 0 #1 if needed
    if hud_alert == AH.FCW:
      forward_collision_warning = hud_alert[1]
      if forward_collision_warning > 1:
        forward_collision_warning = 1
    #cruise state: 0 unavailable, 1 available, 2 enabled, 3 hold
    cc_state = 1 
    speed_limit_to_car = int(self.speedlimit_units)
    alca_state = 0x00 
    
    speed_override = 0
    collision_warning = 0x00
    acc_speed_limit_mph = 0
    speed_control_enabled = 0
    accel_min = -15
    accel_max = 5
    acc_speed_kph = 0
    if enabled:
      #self.opState  0-disabled, 1-enabled, 2-disabling, 3-unavailable, 5-warning
      if self.opState == 0:
        op_status = 0x02
      if self.opState == 1:
        op_status = 0x03
      if self.opState == 2:
        op_status = 0x08
      if self.opState == 3:
        op_status = 0x01
      if self.opState == 5:
        op_status = 0x03
      alca_state = 0x08 + turn_signal_needed
      #canceled by user
      if self.ALCA.laneChange_cancelled and (self.ALCA.laneChange_cancelled_counter > 0):
        alca_state = 0x14
      #min speed for ALCA
      if CS.CL_MIN_V > CS.v_ego:
        alca_state = 0x05
      if not enable_steer_control:
        #op_status = 0x08
        hands_on_state = 0x02
      if hud_alert == AH.STEER:
        if snd_chime == CM.MUTE:
          hands_on_state = 0x03
        else:
          hands_on_state = 0x05
      acc_speed_limit_mph = max(self.ACC.acc_speed_kph * CV.KPH_TO_MPH,1)
      if CS.pedal_interceptor_available:
        acc_speed_limit_mph = max(self.PCC.pedal_speed_kph * CV.KPH_TO_MPH,1)
        acc_speed_kph = self.PCC.pedal_speed_kph
      if hud_alert == AH.FCW:
        collision_warning = hud_alert[1]
        if collision_warning > 1:
          collision_warning = 1
        #use disabling for alerts/errors to make them aware someting is goin going on
      if (snd_chime == CM.DOUBLE) or (hud_alert == AH.FCW):
        op_status = 0x08
      if self.ACC.enable_adaptive_cruise:
        acc_speed_kph = self.ACC.new_speed #pcm_speed * CV.MS_TO_KPH
      if (CS.pedal_interceptor_available and self.PCC.enable_pedal_cruise) or (self.ACC.enable_adaptive_cruise):
        speed_control_enabled = 1
        cc_state = 2
      else:
        if (CS.pcm_acc_status == 4):
          #car CC enabled but not OP, display the HOLD message
          cc_state = 3
    send_fake_msg = False
    send_fake_warning = False

    if enabled:
      if frame % 2 == 0:
        send_fake_msg = True
      if frame % 25 == 0:
        send_fake_warning = True
    else:
      if frame % 23 == 0:
        send_fake_msg = True
      if frame % 60 == 0:
        send_fake_warning = True
    if frame % 10 == 0:
      can_sends.append(teslacan.create_fake_DAS_obj_lane_msg(self.leadDx,self.leadDy,self.leadClass,self.rLine,self.lLine,self.curv0,self.curv1,self.curv2,self.curv3,self.laneRange,self.laneWidth))
    speed_override = 0
    if (CS.pedal_interceptor_value > 10) and (cc_state > 1):
      speed_override = 0 #force zero for now
    if (not enable_steer_control) and op_status == 3:
      #hands_on_state = 0x03
      self.DAS_219_lcTempUnavailableSpeed = 1
      self.warningCounter = 100
      self.warningNeeded = 1
    if enabled and self.ALCA.laneChange_cancelled and (not CS.steer_override) and (not CS.blinker_on) and (self.ALCA.laneChange_cancelled_counter > 0): 
      self.DAS_221_lcAborting = 1
      self.warningCounter = 300
      self.warningNeeded = 1
    if send_fake_msg:
      if enable_steer_control and op_status == 3:
        op_status = 0x5
      can_sends.append(teslacan.create_fake_DAS_msg(speed_control_enabled,speed_override,self.DAS_206_apUnavailable, collision_warning, op_status, \
            acc_speed_kph, \
            turn_signal_needed,forward_collision_warning,hands_on_state, \
            cc_state, 1 if (CS.pedal_interceptor_available) else 0,alca_state, \
            #acc_speed_limit_mph,
            CS.v_cruise_pcm * CV.KPH_TO_MPH, 
            speed_limit_to_car,
            apply_angle,
            1 if enable_steer_control else 0))
    if send_fake_warning or (self.opState == 2) or (self.opState == 5) or (self.stopSignWarning != self.stopSignWarning_last) or (self.stopLightWarning != self.stopLightWarning_last) or (self.warningNeeded == 1) or (frame % 100 == 0):
      #if it's time to send OR we have a warning or emergency disable
      can_sends.append(teslacan.create_fake_DAS_warning(self.DAS_211_accNoSeatBelt, CS.DAS_canErrors, \
            self.DAS_202_noisyEnvironment, CS.DAS_doorOpen, CS.DAS_notInDrive, CS.enableDasEmulation, CS.enableRadarEmulation, \
            self.stopSignWarning, self.stopLightWarning, \
            self.DAS_222_accCameraBlind, self.DAS_219_lcTempUnavailableSpeed, self.DAS_220_lcTempUnavailableRoad, self.DAS_221_lcAborting, \
            self.DAS_207_lkasUnavailable,self.DAS_208_rackDetected, self.DAS_025_steeringOverride,self.ldwStatus,0,CS.useWithoutHarness))
      self.stopLightWarning_last = self.stopLightWarning
      self.stopSignWarning_last = self.stopSignWarning
      self.warningNeeded = 0
    # end of DAS emulation """
    if frame % 100 == 0: # and CS.hasTeslaIcIntegration:
        #IF WE HAVE softPanda RUNNING, send a message every second to say we are still awake
        can_sends.append(teslacan.create_fake_IC_msg())
    idx = frame % 16
    cruise_btn = None
    if self.ACC.enable_adaptive_cruise and not CS.pedal_interceptor_available:
      cruise_btn = self.ACC.update_acc(enabled, CS, frame, actuators, pcm_speed, \
                    self.speed_limit_for_cc, self.speedlimit_valid, \
                    self.set_speed_limit_active, self.speed_limit_offset)
      if cruise_btn:
          cruise_msg = teslacan.create_cruise_adjust_msg(
            spdCtrlLvr_stat=cruise_btn,
            turnIndLvr_Stat= 0, #turn_signal_needed,
            real_steering_wheel_stalk=CS.steering_wheel_stalk)
          # Send this CAN msg first because it is racing against the real stalk.
          can_sends.insert(0, cruise_msg)
    apply_accel = 0.
    if CS.pedal_interceptor_available and frame % 5 == 0: # pedal processed at 20Hz
      apply_accel, accel_needed, accel_idx = self.PCC.update_pdl(enabled, CS, frame, actuators, pcm_speed, \
                    self.speed_limit_for_cc * CV.KPH_TO_MS, self.speedlimit_valid, \
                    self.set_speed_limit_active, self.speed_limit_offset * CV.KPH_TO_MS, self.alca_enabled)
      can_sends.append(teslacan.create_pedal_command_msg(apply_accel, int(accel_needed), accel_idx))
    self.last_angle = apply_angle
    self.last_accel = apply_accel
    
    return pedal_can_sends + can_sends
