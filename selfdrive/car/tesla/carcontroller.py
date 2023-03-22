from common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_angle_limits
from selfdrive.car.tesla.teslacan import TeslaCAN

from selfdrive.car.tesla.HUD_module import HUDController
from selfdrive.car.tesla.LONG_module import LONGController
from selfdrive.car.modules.CFG_module import load_bool_param
from opendbc.can.packer import CANPacker
from selfdrive.car.tesla.values import DBC, CAR, CarControllerParams, CAN_CHASSIS, CAN_AUTOPILOT, CAN_EPAS, CruiseButtons
import cereal.messaging as messaging
from common.numpy_fast import clip, interp
from cereal import log
import datetime
from common import realtime
import math


def gen_solution(CS):
  fix = 0
  if CS.gpsAccuracy < 10.:
    fix = 1
  timestamp = int(
    ((datetime.datetime.now() - datetime.datetime(1970, 1, 1)).total_seconds())
    * 1e03
  )
  vN = math.cos(math.radians(CS.gpsHeading)) * CS.gpsVehicleSpeed
  vE = math.sin(math.radians(CS.gpsHeading)) * CS.gpsVehicleSpeed
  gps_fix = {
    "bearingDeg": CS.gpsHeading,  # heading of motion in degrees
    "altitude": CS.gpsElevation,  # altitude above ellipsoid
    "latitude": CS.gpsLatitude,  # latitude in degrees
    "longitude": CS.gpsLongitude,  # longitude in degrees
    "speed": CS.gpsVehicleSpeed,  # ground speed in meters
    "accuracy": CS.gpsAccuracy,  # horizontal accuracy (1 sigma?)
    "unixTimestampMillis": timestamp,  # UTC time in ms since start of UTC stime
    "vNED": [vN, vE, 0.0],  # velocity in NED frame in m/s
    "speedAccuracy": 0.5,  # speed accuracy in m/s
    "verticalAccuracy": 0.5,  # vertical accuracy in meters
    "bearingAccuracyDeg": 0.5,  # heading accuracy in degrees
    "source": "ublox",
    "flags": fix,  # 1 of gpsAccuracy less than 2 meters
  }
  return log.Event.new_message(gpsLocationTesla=gps_fix)

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.frame = 0
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_name)
    self.pt_packer = None
    if DBC[CP.carFingerprint]['pt']:
      self.pt_packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.tesla_can = TeslaCAN(self.packer, self.pt_packer)
    self.prev_das_steeringControl_counter = -1
    self.long_control_counter = 0

    #initialize modules
    
    self.hud_controller = HUDController(CP,self.packer,self.tesla_can)
    pedalcan = 2
    if load_bool_param("TinklaPedalCanZero", False):
      pedalcan = 0
    self.long_controller = LONGController(CP,self.packer,self.tesla_can,pedalcan)


    self.cruiseDelayFrame = 0
    self.prevCruiseEnabled = False

    self.lP = messaging.sub_sock('longitudinalPlan') 
    self.rS = messaging.sub_sock('radarState') 
    self.mD = messaging.sub_sock('modelV2')
    self.cS = messaging.sub_sock('controlsState')
    self.long_control_counter = 0 
    self.gpsLocationTesla = messaging.pub_sock("gpsLocationTesla")

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel
    enabled = CC.enabled
    
    if self.frame % 100 == 0:
      CS.autoresumeAcc = load_bool_param("TinklaAutoResumeACC",False)

    can_sends = []
    #add 1 second delay logic to wait for AP which has a status at 2Hz
    if self.CP.carFingerprint != CAR.PREAP_MODELS and not CS.autopilot_disabled:
      if CS.cruiseEnabled:
        if not self.prevCruiseEnabled:
          self.cruiseDelayFrame = self.frame
        if self.frame - self.cruiseDelayFrame > 30:
          CS.cruiseDelay = True
      else:
        self.cruiseDelayFrame = 0
        CS.cruiseDelay = False
    self.prevCruiseEnabled = CS.cruiseEnabled

    #receive socks
    long_plan = messaging.recv_one_or_none(self.lP)
    radar_state = messaging.recv_one_or_none(self.rS)
    model_data = messaging.recv_one_or_none(self.mD)
    controls_state = messaging.recv_one_or_none(self.cS)

    if not enabled:
      self.v_target = CS.out.vEgo
      self.a_target = 1

    if CS.use_tesla_gps and (self.frame % 10 == 0):
      if self.gpsLocationTesla is None:
          self.gpsLocationTesla = messaging.pub_sock("gpsLocationTesla")
      sol = gen_solution(CS)
      sol.logMonoTime = int(realtime.sec_since_boot() * 1e9)
      self.gpsLocationTesla.send(sol.to_bytes())

    # Cancel when openpilot is not enabled anymore and no autopilot
    # BB: do we need to do this? AP/Tesla does not behave this way
    #   LKAS can be disabled by steering and ACC remains engaged
    #TODO: we need more logic arround this for AP0
    if not enabled and bool(CS.out.cruiseState.enabled) and not CS.enableHumanLongControl:
      pcm_cancel_cmd = True

    if ((self.frame % 10) == 0 and pcm_cancel_cmd):
      stlk_counter = ((CS.msg_stw_actn_req['MC_STW_ACTN_RQ'] + 1) % 16)
      can_sends.insert(0,self.tesla_can.create_action_request(CS.msg_stw_actn_req, CruiseButtons.CANCEL, CAN_CHASSIS[self.CP.carFingerprint],stlk_counter))
      if (self.CP.carFingerprint in [CAR.AP1_MODELS,CAR.AP2_MODELS]):
        can_sends.insert(1,self.tesla_can.create_action_request(CS.msg_stw_actn_req, CruiseButtons.CANCEL, CAN_AUTOPILOT[self.CP.carFingerprint],stlk_counter))

    #now process controls
    if enabled and not CS.human_control:
      # Angular rate limit based on speed
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, self.CCP)

      # To not fault the EPS
      apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - 20, CS.out.steeringAngleDeg + 20)
    else:
      apply_angle = CS.out.steeringAngleDeg

    if (self.frame % self.CCP.STEER_STEP == 0) and (enabled or (self.CP.carFingerprint == CAR.PREAP_MODELS)):
      ldw_haptic = 0
      if CC.hudControl.leftLaneDepart or CC.hudControl.rightLaneDepart:
        ldw_haptic = 1
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, enabled and not CS.human_control and not CS.out.cruiseState.standstill, ldw_haptic, CAN_EPAS[self.CP.carFingerprint], 1))

      self.apply_angle_last = apply_angle

    #update LONG Control module
    can_messages = self.long_controller.update(enabled, CS, self.frame, actuators, pcm_cancel_cmd,CC.cruiseControl.override, long_plan,radar_state)
    if len(can_messages) > 0:
      can_sends[0:0] = can_messages

    #update HUD Integration module
    can_messages = self.hud_controller.update(controls_state, enabled, CS, self.frame, actuators, pcm_cancel_cmd, CC.hudControl.visualAlert, CC.hudControl.audibleAlert,
            CC.hudControl.leftLaneVisible, CC.hudControl.rightLaneVisible, CC.hudControl.leadVisible, CC.hudControl.leftLaneDepart, CC.hudControl.rightLaneDepart,CS.human_control,radar_state,CS.lat_plan,apply_angle,model_data)
    if len(can_messages) > 0:
      can_sends.extend(can_messages)

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    
    return new_actuators, can_sends
