from selfdrive.car.tesla.teslacan import TeslaCAN
from selfdrive.car.tesla.HSO_module import HSOController
from selfdrive.car.tesla.BLNK_module import BLNKController
from selfdrive.car.tesla.ALC_module import ALCController
from selfdrive.car.tesla.HUD_module import HUDController
from selfdrive.car.tesla.LONG_module import LONGController
from opendbc.can.packer import CANPacker
from selfdrive.car.tesla.values import CarControllerParams, CAN_CHASSIS, CAN_AUTOPILOT, CAN_EPAS
import cereal.messaging as messaging
from common.numpy_fast import clip, interp
from common.params import Params

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.last_angle = 0
    self.packer = CANPacker(dbc_name)
    self.tesla_can = TeslaCAN(dbc_name, self.packer)
    self.prev_das_steeringControl_counter = -1
    self.long_control_counter = 0

    #initialize modules
    self.HSO = HSOController(self)
    self.blinker_controller = BLNKController()
    self.alca_controller = ALCController()
    self.hud_controller = HUDController(CP,self.packer,self.tesla_can)
    self.long_controller = LONGController(CP,self.packer,self.tesla_can)


    self.cruiseDelayFrame = 0
    self.prevCruiseEnabled = False

    self.lP = messaging.sub_sock('longitudinalPlan')
    self.rS = messaging.sub_sock('radarState')
    self.laP = messaging.sub_sock('lateralPlan')

  def update(self, enabled, CS, frame, actuators, cruise_cancel, pcm_speed, hud_alert, audible_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):
    #read params once a second

    can_sends = []
    #add 1 second delay logic to wait for AP which has a status at 2Hz
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
    lat_plan = messaging.recv_one_or_none(self.laP)
    
    
    # Update HSO module
    human_control,hands_on_fault,lkas_enabled = self.HSO.update_stat(self, CS, enabled, actuators, frame)
    
    #update blinker tap module
    self.blinker_controller.update_state(CS, frame)
    CS.tap_direction = self.blinker_controller.tap_direction

    #update ALCA module
    self.alca_controller.update(enabled, CS, frame, lat_plan)

    if not enabled:
      self.v_target = CS.out.vEgo
      self.a_target = 1

    if (hands_on_fault):
      enabled = False

    # Cancel when openpilot is not enabled anymore and no autopilot
    # BB: do we need to do this? AP/Tesla does not behave this way
    #   LKAS can be disabled by steering and ACC remains engaged
    if not enabled and bool(CS.out.cruiseState.enabled):
      cruise_cancel = True

    if ((frame % 10) == 0 and cruise_cancel):
      # Spam every possible counter value, otherwise it might not be accepted
      for counter in range(16):
        can_sends.append(self.tesla_can.create_action_request(CS.msg_stw_actn_req, cruise_cancel, CAN_CHASSIS[self.CP.carFingerprint],counter))
        if CAN_AUTOPILOT[self.CP.carFingerprint] != -1:
          can_sends.append(self.tesla_can.create_action_request(CS.msg_stw_actn_req, cruise_cancel, CAN_AUTOPILOT[self.CP.carFingerprint],counter))

    #now process controls
    if lkas_enabled and not human_control:
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

    if enabled:
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, lkas_enabled  and not human_control and not CS.out.cruiseState.standstill, CAN_EPAS[self.CP.carFingerprint], 1))


    #update LONG Control module
    can_messages = self.long_controller.update(enabled, CS, frame, actuators, cruise_cancel,pcm_speed,long_plan,radar_state)
    can_sends.extend(can_messages)

    #update HUD Integration module
    can_messages = self.hud_controller.update(enabled, CS, frame, actuators, cruise_cancel, hud_alert, audible_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart,human_control,radar_state,lat_plan,apply_angle)
    can_sends.extend(can_messages)
    
    return can_sends
