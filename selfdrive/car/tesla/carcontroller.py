from selfdrive.car.tesla.teslacan import TeslaCAN

from selfdrive.car.tesla.HUD_module import HUDController
from selfdrive.car.tesla.LONG_module import LONGController
from selfdrive.car.modules.CFG_module import load_bool_param
from opendbc.can.packer import CANPacker
from selfdrive.car.tesla.values import DBC, CAR, CarControllerParams, CAN_CHASSIS, CAN_AUTOPILOT, CAN_EPAS, CruiseButtons
import cereal.messaging as messaging
from common.numpy_fast import clip, interp


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.last_angle = 0
    self.packer = CANPacker(dbc_name)
    self.pt_packer = None
    if DBC[CP.carFingerprint]['pt']:
      self.pt_packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.tesla_can = TeslaCAN(dbc_name, self.packer, self.pt_packer)
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
    
  
  #def update(self, c, enabled, CS, frame, actuators, cruise_cancel):
  def update(self, c, enabled, CS, frame, actuators, cruise_cancel, pcm_speed, pcm_override, hud_alert, audible_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart): 
    if frame % 100 == 0:
      CS.autoresumeAcc = load_bool_param("TinklaAutoResumeACC",False)
    can_sends = []
    #add 1 second delay logic to wait for AP which has a status at 2Hz
    if self.CP.carFingerprint != CAR.PREAP_MODELS and not CS.autopilot_disabled:
      if CS.cruiseEnabled:
        if not self.prevCruiseEnabled:
          self.cruiseDelayFrame = frame
        if frame - self.cruiseDelayFrame > 30:
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

    # Cancel when openpilot is not enabled anymore and no autopilot
    # BB: do we need to do this? AP/Tesla does not behave this way
    #   LKAS can be disabled by steering and ACC remains engaged
    #TODO: we need more logic arround this for AP0
    if not enabled and bool(CS.out.cruiseState.enabled) and not CS.enableHumanLongControl:
      cruise_cancel = True

    if ((frame % 10) == 0 and cruise_cancel):
      stlk_counter = ((CS.msg_stw_actn_req['MC_STW_ACTN_RQ'] + 1) % 16)
      can_sends.insert(0,self.tesla_can.create_action_request(CS.msg_stw_actn_req, CruiseButtons.CANCEL, CAN_CHASSIS[self.CP.carFingerprint],stlk_counter))
      if (self.CP.carFingerprint in [CAR.AP1_MODELS,CAR.AP2_MODELS]):
        can_sends.insert(1,self.tesla_can.create_action_request(CS.msg_stw_actn_req, CruiseButtons.CANCEL, CAN_AUTOPILOT[self.CP.carFingerprint],stlk_counter))

    #now process controls
    if enabled and not CS.human_control:
      apply_angle = actuators.steeringAngleDeg

      # Angular rate limit based on speed
      steer_up = (self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle))
      rate_limit = CarControllerParams.RATE_LIMIT_UP if steer_up else CarControllerParams.RATE_LIMIT_DOWN
      max_angle_diff = interp(CS.out.vEgo, rate_limit.speed_points, rate_limit.max_angle_diff_points)
      apply_angle = clip(apply_angle, (self.last_angle - max_angle_diff), (self.last_angle + max_angle_diff))

      # To not fault the EPS
      apply_angle = clip(apply_angle, (CS.out.steeringAngleDeg - 20), (CS.out.steeringAngleDeg + 20))
    else:
      apply_angle = CS.out.steeringAngleDeg

    self.last_angle = apply_angle

    if enabled or (self.CP.carFingerprint == CAR.PREAP_MODELS):
      ldw_haptic = 0
      if left_lane_depart or right_lane_depart:
        ldw_haptic = 1
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, enabled and not CS.human_control and not CS.out.cruiseState.standstill, ldw_haptic, CAN_EPAS[self.CP.carFingerprint], 1))


    #update LONG Control module
    can_messages = self.long_controller.update(enabled, CS, frame, actuators, cruise_cancel,pcm_speed,pcm_override, long_plan,radar_state)
    if len(can_messages) > 0:
      can_sends[0:0] = can_messages

    #update HUD Integration module
    can_messages = self.hud_controller.update(controls_state, enabled, CS, frame, actuators, cruise_cancel, hud_alert, audible_alert,
            left_line, right_line, lead, left_lane_depart, right_lane_depart,CS.human_control,radar_state,CS.lat_plan,apply_angle,model_data)
    if len(can_messages) > 0:
      can_sends.extend(can_messages)

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle
    
    return new_actuators, can_sends
