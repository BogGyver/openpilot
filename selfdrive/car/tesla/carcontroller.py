from common.numpy_fast import clip, interp
from selfdrive.car.tesla.teslacan import TeslaCAN
from selfdrive.car.tesla.HSO_module import HSOController
from selfdrive.car.tesla.BLNK_module import Blinker
from opendbc.can.packer import CANPacker
from selfdrive.car.tesla.values import CarControllerParams, CAR, CAN_CHASSIS, CAN_AUTOPILOT, CAN_EPAS, CAN_POWERTRAIN
import cereal.messaging as messaging
from cereal import log

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

def _is_present(lead):
  return bool((not (lead is None)) and (lead.dRel > 0))

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.last_angle = 0
    self.packer = CANPacker(dbc_name)
    self.tesla_can = TeslaCAN(dbc_name, self.packer)
    self.prev_das_steeringControl_counter = -1
    self.long_control_counter = 0
    self.HSO = HSOController(self)
    self.blinker = Blinker()
    self.laP = messaging.sub_sock('lateralPlan')
    self.alca_engaged_frame = 0
    if CP.openpilotLongitudinalControl:
      self.lP = messaging.sub_sock('longitudinalPlan')
      self.rS = messaging.sub_sock('radarState')
      self.v_target = None
      self.lead_1 = None
      self.long_control_counter = 1


  def update(self, enabled, CS, frame, actuators, cruise_cancel):
    can_sends = []

    # Temp disable steering on a hands_on_fault, and allow for user override
    # TODO: better user blending
    hands_on_fault = (CS.steer_warning == "EAC_ERROR_HANDS_ON" and CS.hands_on_level >= 3)
    lkas_enabled = enabled and (not hands_on_fault)

    # Update modules
    human_control = self.HSO.update_stat(self, CS, enabled, actuators, frame)
    self.blinker.update_state(CS, frame)

    #get lat plan info
    lat_plan = messaging.recv_one_or_none(self.laP)
    if lat_plan is not None:
      CS.alca_pre_engage = lat_plan.lateralPlan.laneChangeState in [LaneChangeState.preLaneChange]
      CS.alca_engaged = lat_plan.lateralPlan.laneChangeState in [LaneChangeState.laneChangeStarting,
                                                  LaneChangeState.laneChangeFinishing]
      CS.alca_direction = lat_plan.lateralPlan.laneChangeDirection # 0-none, 1-left, 2-right 
    if CS.alca_pre_engage:
      if CS.alca_pre_engage != CS.prev_alca_pre_engage:
        self.alca_engaged_frame = frame
      if (CS.autoStartAlcaDelay > 0) and (self.alca_engaged_frame > 0) and (frame - self.alca_engaged_frame > CS.autoStartAlcaDelay * 100):
        CS.alca_need_engagement = True
      else:
        CS.alca_need_engagement = False
    else:
      CS.alca_need_engagement = False
      self.alca_engaged_frame = frame
    self.prev_alca_pre_engage = CS.alca_pre_engage

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

    #only send the DAS_steeringControl after we received a new counter for it
    if enabled:
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, lkas_enabled, CAN_EPAS[self.CP.carFingerprint], 1))

    if enabled and self.CP.openpilotLongitudinalControl and (frame %2 == 0):
      #we use the same logic from planner here to get the speed
      long_plan = messaging.recv_one_or_none(self.lP)
      radar_state = messaging.recv_one_or_none(self.rS)
      if radar_state is not None:
        self.lead_1 = radar_state.radarState.leadOne
      if long_plan is not None:
        self.v_target = long_plan.longitudinalPlan.vTargetFuture # to try vs vTarget
        self.a_target = abs(long_plan.longitudinalPlan.aTarget) # to try vs aTarget
      if self.v_target is None:
        self.v_target = CS.out.vEgo
        self.a_target = 1
      following = False
      #TODO: see what works best for these
      tesla_accel_limits = [-2*self.a_target,self.a_target]
      tesla_jerk_limits = [-4*self.a_target,2*self.a_target]
      #if _is_present(self.lead_1):
      #  following = self.lead_1.status and self.lead_1.dRel < 45.0 and self.lead_1.vLeadK > CS.out.vEgo and self.lead_1.aLeadK > 0.0
      
      #we now create the DAS_control for AP1 or DAS_longControl for AP2
      if self.CP.carFingerprint == CAR.AP2_MODELS:
        can_sends.append(self.tesla_can.create_ap2_long_control(self.v_target, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))
      if self.CP.carFingerprint == CAR.AP1_MODELS:
        can_sends.append(self.tesla_can.create_ap1_long_control(self.v_target, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

    if (not enabled) and self.CP.openpilotLongitudinalControl and (frame %2 == 0):
      #send this values so we can enable at 0 km/h
      tesla_accel_limits = [-1.4000000000000004,1.8000000000000007]
      tesla_jerk_limits = [-0.46000000000000085,0.47600000000000003]
      if self.CP.carFingerprint == CAR.AP2_MODELS:
        can_sends.append(self.tesla_can.create_ap2_long_control(350.0/3.6, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))
      if self.CP.carFingerprint == CAR.AP1_MODELS:
        can_sends.append(self.tesla_can.create_ap1_long_control(350.0/3.6, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

    
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
      

    # TODO: HUD control: Autopilot Status, (Status2 also needed for long control),
    #       Lanes and BodyControls (keep turn signal on during ALCA)

    return can_sends
