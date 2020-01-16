import datetime
from cereal import log,tesla
from common.params import Params
from collections import namedtuple
from common.numpy_fast import clip, interp
from common import realtime
from selfdrive.car.tesla import teslacan
from selfdrive.car.tesla.blinker_module import Blinker
from selfdrive.car.tesla.speed_utils.fleet_speed import FleetSpeed
from selfdrive.car.tesla.values import AH, CM
from selfdrive.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from selfdrive.car.modules.ALCA_module import ALCAController
from selfdrive.car.modules.GYRO_module import GYROController
from selfdrive.car.tesla.ACC_module import ACCController
from selfdrive.car.tesla.PCC_module import PCCController
from selfdrive.car.tesla.HSO_module import HSOController
from selfdrive.car.tesla.speed_utils.movingaverage import MovingAverage
from selfdrive.car.tesla.AHB_module import AHBController
import selfdrive.messaging as messaging

# Steer angle limits
ANGLE_MAX_BP = [0., 27., 36.]
ANGLE_MAX_V = [410., 92., 36.]

ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .25]     # windup limit
ANGLE_DELTA_VU = [5., 3.5, 0.8]   # unwind limit
#LDW WARNING LEVELS
LDW_WARNING_1 = 0.9
LDW_WARNING_2 = 0.5
LDW_LANE_PROBAB = 0.2

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



class CarController():
  def __init__(self, dbc_name):
    self.fleet_speed_state = 0
    self.cc_counter = 0
    self.alcaStateData = None
    self.icLeadsData = None
    self.params = Params()
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.packer = CANPacker(dbc_name)
    self.epas_disabled = True
    self.last_angle = 0.
    self.last_accel = 0.
    self.blinker = Blinker()
    self.ALCA = ALCAController(self,True,True)  # Enabled and SteerByAngle both True
    self.ACC = ACCController(self)
    self.PCC = PCCController(self)
    self.HSO = HSOController(self)
    self.GYRO = GYROController()
    self.AHB = AHBController(self)
    self.sent_DAS_bootID = False
    self.speedlimit = None
    self.trafficevents = messaging.sub_sock('trafficEvents', conflate=True)
    self.pathPlan = messaging.sub_sock('pathPlan', conflate=True)
    self.radarState = messaging.sub_sock('radarState', conflate=True)
    self.icLeads = messaging.sub_sock('uiIcLeads', conflate=True)
    self.icCarLR = messaging.sub_sock('uiIcCarLR', conflate=True)
    self.alcaState = messaging.sub_sock('alcaState', conflate=True)
    self.gpsLocationExternal = None 
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
    self.speed_limit_ms = 0.

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
    self.laneRange = 30  #max is 160m but OP has issues with precision beyond 50

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
    self.LDW_ENABLE_SPEED = 16 
    self.should_ldw = False
    self.ldw_numb_frame_end = 0

    self.isMetric = (self.params.get("IsMetric") == "1")

    self.ahbLead1 = None

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
        self.speedlimit = messaging.sub_sock('liveMapData', conflate=True)


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
      print ("INVALID HUD", hud)
      hud = HUDData(0xc6, 255, 64, 0xc0, 209, 0x40, 0, 0, 0, 0)

    # **** process the car messages ****

    # *** compute control surfaces ***

    # Prevent steering while stopped
    MIN_STEERING_VEHICLE_VELOCITY = 0.05 # m/s
    vehicle_moving = (CS.v_ego >= MIN_STEERING_VEHICLE_VELOCITY)

    #upodate custom UI buttons and alerts
    CS.UE.update_custom_ui()
      
    if (frame % 100 == 0):
      CS.cstm_btns.send_button_info()
      #read speed limit params
      if CS.hasTeslaIcIntegration:
        self.set_speed_limit_active = True
        self.speed_limit_offset = CS.userSpeedLimitOffsetKph
      else:
        self.set_speed_limit_active = (self.params.get("SpeedLimitOffset") is not None) and (self.params.get("LimitSetSpeed") == "1")
        if self.set_speed_limit_active:
          self.speed_limit_offset = float(self.params.get("SpeedLimitOffset"))
          if not self.isMetric:
            self.speed_limit_offset = self.speed_limit_offset * CV.MPH_TO_MS
        else:
          self.speed_limit_offset = 0.
    if CS.useTeslaGPS and (frame % 10 == 0):
      if self.gpsLocationExternal is None:
        self.gpsLocationExternal = messaging.pub_sock('gpsLocationExternal')
      sol = gen_solution(CS)
      sol.logMonoTime = int(realtime.sec_since_boot() * 1e9)
      self.gpsLocationExternal.send(sol.to_bytes())

    #get pitch/roll/yaw every 0.1 sec
    if (frame %10 == 0):
      (self.accPitch, self.accRoll, self.accYaw),(self.magPitch, self.magRoll, self.magYaw),(self.gyroPitch, self.gyroRoll, self.gyroYaw) = self.GYRO.update(CS.v_ego,CS.a_ego,CS.angle_steers)
      CS.UE.uiGyroInfoEvent(self.accPitch, self.accRoll, self.accYaw,self.magPitch, self.magRoll, self.magYaw,self.gyroPitch, self.gyroRoll, self.gyroYaw)

    # Update statuses for custom buttons every 0.1 sec.
    if (frame % 10 == 0):
      self.ALCA.update_status((CS.cstm_btns.get_button_status("alca") > 0) and ((CS.enableALCA and not CS.hasTeslaIcIntegration) or (CS.hasTeslaIcIntegration and CS.alcaEnabled)))

    self.blinker.update_state(CS, frame)

    # update PCC module info
    pedal_can_sends = self.PCC.update_stat(CS, frame)
    if self.PCC.pcc_available:
      self.ACC.enable_adaptive_cruise = False
    else:
      # Update ACC module info.
      self.ACC.update_stat(CS, True)
      self.PCC.enable_pedal_cruise = False

    # update CS.v_cruise_pcm based on module selected.
    speed_uom_kph = 1.
    if CS.imperial_speed_units:
      speed_uom_kph = CV.KPH_TO_MPH
    if self.ACC.enable_adaptive_cruise:
      CS.v_cruise_pcm = self.ACC.acc_speed_kph * speed_uom_kph
    elif self.PCC.enable_pedal_cruise:
      CS.v_cruise_pcm = self.PCC.pedal_speed_kph * speed_uom_kph
    else:
      CS.v_cruise_pcm = max(0., CS.v_ego * CV.MS_TO_KPH) * speed_uom_kph
    self.alca_enabled = self.ALCA.update(enabled, CS, actuators, self.alcaStateData, frame, self.blinker)
    self.should_ldw = self._should_ldw(CS, frame)
    apply_angle = -actuators.steerAngle  # Tesla is reversed vs OP.
    # Update HSO module info.
    human_control = self.HSO.update_stat(self,CS, enabled, actuators, frame)
    human_lane_changing = CS.turn_signal_stalk_state > 0 and not self.alca_enabled
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

    #BB disable limits to test 0.5.8
    # apply_angle = clip(apply_angle , self.last_angle - angle_rate_lim, self.last_angle + angle_rate_lim) 
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
    # DAS_turn_signal_request (2),DAS_forward_collision_warning (2), DAS_hands_on_state (3), 
    # DAS_cc_state (2), DAS_usingPedal(1),DAS_alca_state (5),
    # DAS_acc_speed_limit (8),
    # DAS_speed_limit_units(8)
    #send fake_das data as 0x553
    # TODO: forward collision warning

    if frame % 10 == 0:
        speedlimitMsg = None
        if self.speedlimit is not None:
          speedlimitMsg = messaging.recv_one_or_none(self.speedlimit)
        icLeadsMsg = self.icLeads.receive(non_blocking=True)
        radarStateMsg = messaging.recv_one_or_none(self.radarState)
        alcaStateMsg = self.alcaState.receive(non_blocking=True)
        pathPlanMsg = messaging.recv_one_or_none(self.pathPlan)
        icCarLRMsg = self.icCarLR.receive(non_blocking=True)
        trafficeventsMsgs = None
        if self.trafficevents is not None:
          trafficeventsMsgs = messaging.recv_sock(self.trafficevents)
        if CS.hasTeslaIcIntegration:
          self.speed_limit_ms = CS.speed_limit_ms
        if (speedlimitMsg is not None) and not CS.useTeslaMapData:
          lmd = speedlimitMsg.liveMapData
          self.speed_limit_ms = lmd.speedLimit if lmd.speedLimitValid else 0
        if icLeadsMsg is not None:
          self.icLeadsData = tesla.ICLeads.from_bytes(icLeadsMsg)
        if radarStateMsg is not None:
          #to show lead car on IC
          if self.icLeadsData is not None:
            can_messages = self.showLeadCarOnICCanMessage(radarStateMsg = radarStateMsg)
            can_sends.extend(can_messages)
        if alcaStateMsg is not None:
          self.alcaStateData =  tesla.ALCAState.from_bytes(alcaStateMsg)
        if pathPlanMsg is not None:
          #to show curvature and lanes on IC
          if self.alcaStateData is not None:
            self.handlePathPlanSocketForCurvatureOnIC(pathPlanMsg = pathPlanMsg, alcaStateData = self.alcaStateData,CS = CS)
        if icCarLRMsg is not None:
          can_messages = self.showLeftAndRightCarsOnICCanMessages(icCarLRMsg = tesla.ICCarsLR.from_bytes(icCarLRMsg))
          can_sends.extend(can_messages)
        if trafficeventsMsgs is not None:
          can_messages = self.handleTrafficEvents(trafficEventsMsgs = trafficeventsMsgs)
          can_sends.extend(can_messages)

    op_status = 0x02
    hands_on_state = 0x00
    forward_collision_warning = 0 #1 if needed
    if hud_alert == AH.FCW:
      forward_collision_warning = hud_alert[1]
      if forward_collision_warning > 1:
        forward_collision_warning = 1
    #cruise state: 0 unavailable, 1 available, 2 enabled, 3 hold
    cc_state = 1 
    alca_state = 0x00 
    
    speed_override = 0
    collision_warning = 0x00
    speed_control_enabled = 0
    accel_min = -15
    accel_max = 5
    acc_speed_kph = 0
    send_fake_warning = False
    send_fake_msg = False
    if enabled:
      #self.opState  0-disabled, 1-enabled, 2-disabling, 3-unavailable, 5-warning
      alca_state = 0x01
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
      if self.blinker.override_direction > 0:
        alca_state = 0x08 + self.blinker.override_direction
      elif (self.lLine > 1) and (self.rLine > 1):
        alca_state = 0x08
      elif (self.lLine > 1):
        alca_state = 0x06
      elif (self.rLine > 1): 
        alca_state = 0x07
      else:
        alca_state = 0x01
      #canceled by user
      if self.ALCA.laneChange_cancelled and (self.ALCA.laneChange_cancelled_counter > 0):
        alca_state = 0x14
      #min speed for ALCA
      if (CS.CL_MIN_V > CS.v_ego):
        alca_state = 0x05
      #max angle for ALCA
      if (abs(actuators.steerAngle) >= CS.CL_MAX_A):
        alca_state = 0x15
      if not enable_steer_control:
        #op_status = 0x08
        hands_on_state = 0x02
      if hud_alert == AH.STEER:
        if snd_chime == CM.MUTE:
          hands_on_state = 0x03
        else:
          hands_on_state = 0x05
      if self.PCC.pcc_available:
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
      if (self.PCC.pcc_available and self.PCC.enable_pedal_cruise) or (self.ACC.enable_adaptive_cruise):
        speed_control_enabled = 1
        cc_state = 2
        if not self.ACC.adaptive:
            cc_state = 3
        CS.speed_control_enabled = 1
      else:
        CS.speed_control_enabled = 0
        if (CS.pcm_acc_status == 4):
          #car CC enabled but not OP, display the HOLD message
          cc_state = 3
    else:
      if (CS.pcm_acc_status == 4):
        cc_state = 3
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
    if enabled and self.ALCA.laneChange_cancelled and (not CS.steer_override) and (CS.turn_signal_stalk_state == 0) and (self.ALCA.laneChange_cancelled_counter > 0):
      self.DAS_221_lcAborting = 1
      self.warningCounter = 300
      self.warningNeeded = 1
    if CS.hasTeslaIcIntegration:
      highLowBeamStatus,highLowBeamReason,ahbIsEnabled = self.AHB.update(CS,frame,self.ahbLead1)
      if frame % 5 == 0:
        self.cc_counter = (self.cc_counter + 1) % 40 #use this to change status once a second
        self.fleet_speed_state = 0x00 #fleet speed unavailable
        if FleetSpeed.is_available(CS):
          if self.ACC.fleet_speed.is_active(frame) or self.PCC.fleet_speed.is_active(frame):
            self.fleet_speed_state = 0x02 #fleet speed enabled
          else:
            self.fleet_speed_state = 0x01 #fleet speed available
        can_sends.append(teslacan.create_fake_DAS_msg2(highLowBeamStatus,highLowBeamReason,ahbIsEnabled,self.fleet_speed_state))
    if (self.cc_counter < 3) and (self.fleet_speed_state == 0x02):
      CS.v_cruise_pcm = CS.v_cruise_pcm + 1 
      send_fake_msg = True
    if (self.cc_counter == 3):
      send_fake_msg = True
    if send_fake_msg:
      if enable_steer_control and op_status == 3:
        op_status = 0x5
      park_brake_request = int(CS.ahbEnabled)
      if park_brake_request == 1:
        print("Park Brake Request received")
      adaptive_cruise = 1 if (not self.PCC.pcc_available and self.ACC.adaptive) or self.PCC.pcc_available else 0
      can_sends.append(teslacan.create_fake_DAS_msg(speed_control_enabled,speed_override,self.DAS_206_apUnavailable, collision_warning, op_status, \
            acc_speed_kph, \
            self.blinker.override_direction,forward_collision_warning, adaptive_cruise,  hands_on_state, \
            cc_state, 1 if self.PCC.pcc_available else 0, alca_state, \
            CS.v_cruise_pcm,
            CS.DAS_fusedSpeedLimit,
            apply_angle,
            1 if enable_steer_control else 0,
            park_brake_request))
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

    # send enabled ethernet every 0.2 sec
    if frame % 20 == 0:
        can_sends.append(teslacan.create_enabled_eth_msg(1))
    if (not self.PCC.pcc_available) and frame % 5 == 0: # acc processed at 20Hz
      cruise_btn = self.ACC.update_acc(enabled, CS, frame, actuators, pcm_speed, \
                    self.speed_limit_ms * CV.MS_TO_KPH,
                    self.set_speed_limit_active, self.speed_limit_offset)
      if cruise_btn:
          cruise_msg = teslacan.create_cruise_adjust_msg(
            spdCtrlLvr_stat=cruise_btn,
            turnIndLvr_Stat= 0,
            real_steering_wheel_stalk=CS.steering_wheel_stalk)
          # Send this CAN msg first because it is racing against the real stalk.
          can_sends.insert(0, cruise_msg)
    apply_accel = 0.
    if self.PCC.pcc_available and frame % 5 == 0: # pedal processed at 20Hz
      apply_accel, accel_needed, accel_idx = self.PCC.update_pdl(enabled, CS, frame, actuators, pcm_speed, \
                    self.speed_limit_ms,
                    self.set_speed_limit_active, self.speed_limit_offset * CV.KPH_TO_MS, self.alca_enabled)
      can_sends.append(teslacan.create_pedal_command_msg(apply_accel, int(accel_needed), accel_idx))
    self.last_angle = apply_angle
    self.last_accel = apply_accel
    
    return pedal_can_sends + can_sends

  #to show lead car on IC
  def showLeadCarOnICCanMessage(self, radarStateMsg):
    messages = []
    leads = radarStateMsg.radarState
    if leads is None:
      self.ahbLead1 = None
      return messages
    lead_1 = leads.leadOne
    lead_2 = leads.leadTwo
    if (lead_1 is not None) and lead_1.status:
      self.ahbLead1 = lead_1
      self.leadDx = lead_1.dRel
      self.leadDy = self.curv0-lead_1.yRel
      self.leadId = self.icLeadsData.lead1trackId
      self.leadClass = self.icLeadsData.lead1oClass 
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
      self.lead2Id = self.icLeadsData.lead2trackId
      self.lead2Class = self.icLeadsData.lead2oClass 
      self.lead2Vx = lead_2.vRel
      if (self.lead2Id <= 0) or (self.lead2Id == 63):
        self.leadId = 62
    else:
      self.lead2Dx = 0.
      self.lead2Dy = 0.
      self.lead2Class = 0
      self.lead2Id = 0
      self.lead2Vx = 0xF
    messages.append(teslacan.create_DAS_LR_object_msg(0,self.leadClass, self.leadId,
          self.leadDx,self.leadDy,self.leadVx,self.lead2Class,
          self.lead2Id,self.lead2Dx,self.lead2Dy,self.lead2Vx))
    return messages

  def handlePathPlanSocketForCurvatureOnIC(self, pathPlanMsg, alcaStateData, CS):
    pp = pathPlanMsg.pathPlan
    if pp.paramsValid:
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
      self.curv0 = -clip(pp.dPoly[3],-3.5,3.5) #self.curv0Matrix.add(-clip(pp.cPoly[3],-3.5,3.5))
      self.curv1 = -clip(pp.dPoly[2],-0.2,0.2) #self.curv1Matrix.add(-clip(pp.cPoly[2],-0.2,0.2))
      self.curv2 = -clip(pp.dPoly[1],-0.0025,0.0025) #self.curv2Matrix.add(-clip(pp.cPoly[1],-0.0025,0.0025))
      self.curv3 = -clip(pp.dPoly[0],-0.00003,0.00003) #self.curv3Matrix.add(-clip(pp.cPoly[0],-0.00003,0.00003))
      self.laneWidth = pp.laneWidth
      self.visionCurvC0 = self.curv0
      self.prev_ldwStatus = self.ldwStatus
      self.ldwStatus = 0
      if alcaStateData.alcaEnabled:
        #exagerate position a little during ALCA to make lane change look smoother on IC
        self.curv1 = 0.0 #straighten the turn for ALCA
        self.curv0 = -self.ALCA.laneChange_direction * alcaStateData.alcaLaneWidth * alcaStateData.alcaStep / alcaStateData.alcaTotalSteps #animas late change on IC
        self.curv0 = clip(self.curv0, -3.5, 3.5)
        self.lLine = 3
        self.rLine = 3
      else:
        if self.should_ldw:
          if pp.lProb > LDW_LANE_PROBAB:
            lLaneC0 = -pp.lPoly[3]
            if abs(lLaneC0) < LDW_WARNING_2:
              self.ldwStatus = 3
            elif  abs(lLaneC0) < LDW_WARNING_1:
              self.ldwStatus = 1
          if pp.rProb > LDW_LANE_PROBAB:
            rLaneC0 = -pp.rPoly[3]
            if abs(rLaneC0) < LDW_WARNING_2:
              self.ldwStatus = 4
            elif  abs(rLaneC0) < LDW_WARNING_1:
              self.ldwStatus = 2
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

  # Generates IC messages for the Left and Right radar identified cars from radard
  def showLeftAndRightCarsOnICCanMessages(self, icCarLRMsg):
    messages = []
    icCarLR_msg = icCarLRMsg
    if icCarLR_msg is not None:
      #for icCarLR_msg in icCarLR_list:
      messages.append(teslacan.create_DAS_LR_object_msg(1,icCarLR_msg.v1Type,icCarLR_msg.v1Id,
          icCarLR_msg.v1Dx,icCarLR_msg.v1Dy,icCarLR_msg.v1Vrel,icCarLR_msg.v2Type,
          icCarLR_msg.v2Id,icCarLR_msg.v2Dx,icCarLR_msg.v2Dy,icCarLR_msg.v2Vrel))
      messages.append(teslacan.create_DAS_LR_object_msg(2,icCarLR_msg.v3Type,icCarLR_msg.v3Id,
          icCarLR_msg.v3Dx,icCarLR_msg.v3Dy,icCarLR_msg.v3Vrel,icCarLR_msg.v4Type,
          icCarLR_msg.v4Id,icCarLR_msg.v4Dx,icCarLR_msg.v4Dy,icCarLR_msg.v4Vrel))
    return messages

  def handleTrafficEvents(self, trafficEventsMsgs):
    messages = []
    self.reset_traffic_events()
    tr_ev_list = trafficEventsMsgs
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
          messages.append(teslacan.create_fake_DAS_sign_msg(self.roadSignType,self.roadSignStopDist,self.roadSignColor,self.roadSignControlActive))
    return messages

  def _should_ldw(self, CS, frame):
    if not CS.enableLdw:
      return False
    if CS.prev_turn_signal_blinking and not CS.turn_signal_blinking:
      self.ldw_numb_frame_end = frame + int(100 * CS.ldwNumbPeriod)

    return CS.v_ego >= self.LDW_ENABLE_SPEED and not CS.turn_signal_blinking and frame > self.ldw_numb_frame_end
