from common.numpy_fast import clip, interp
from selfdrive.car.tesla.teslacan import TeslaCAN
from selfdrive.car.tesla.HSO_module import HSOController
from selfdrive.car.tesla.BLNK_module import Blinker
from opendbc.can.packer import CANPacker
from selfdrive.car.tesla.values import CarControllerParams, CAR, CAN_CHASSIS, CAN_AUTOPILOT, CAN_EPAS, CAN_POWERTRAIN
import cereal.messaging as messaging
from cereal import log, car
import numpy as np
from selfdrive.config import Conversions as CV

IC_LANE_SCALE = 0.5
LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert

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
    self.IC_integration_counter = 0
    self.IC_integration_warning_counter = 0
    self.IC_previous_enabled = False

    self.cruiseDelayFrame = 0
    self.prevCruiseEnabled = False

    if CP.openpilotLongitudinalControl:
      self.lP = messaging.sub_sock('longitudinalPlan')
      self.rS = messaging.sub_sock('radarState')
      self.v_target = None
      self.lead_1 = None
      self.long_control_counter = 1


  def update(self, enabled, CS, frame, actuators, cruise_cancel, hud_alert, audible_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):
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

    self.IC_integration_counter = ((self.IC_integration_counter + 2) % 100)
    if self.IC_integration_warning_counter > 0:
      self.IC_integration_warning_counter = self.IC_integration_warning_counter - 1
    
    # Update modules
    human_control = self.HSO.update_stat(self, CS, enabled, actuators, frame)
    self.blinker.update_state(CS, frame)

    # Temp disable steering on a hands_on_fault, and allow for user override
    # TODO: better user blending
    hands_on_fault = (CS.hands_on_level >= 2 and not human_control) #and CS.steer_warning == "EAC_ERROR_HANDS_ON"
    lkas_enabled = enabled and (not hands_on_fault)

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

    #get lat plan info
    lat_plan = messaging.recv_one_or_none(self.laP)
    if lat_plan is not None:
      CS.alca_pre_engage = lat_plan.lateralPlan.laneChangeState in [LaneChangeState.preLaneChange]
      CS.alca_engaged = lat_plan.lateralPlan.laneChangeState in [LaneChangeState.laneChangeStarting,
                                                  LaneChangeState.laneChangeFinishing]
      CS.alca_done = lat_plan.lateralPlan.laneChangeState in [LaneChangeState.laneChangeFinishing]
      # 0-none, 1-left, 2-right 
      if (CS.alca_pre_engage or CS.alca_engaged) and not CS.alca_done:
        if lat_plan.lateralPlan.laneChangeDirection == log.LateralPlan.LaneChangeDirection.left:
          CS.alca_direction = 1
        elif lat_plan.lateralPlan.laneChangeDirection == log.LateralPlan.LaneChangeDirection.right:
          CS.alca_direction = 2
        else:
          CS.alca_direction = 0
      else: 
        CS.alca_direction = 0
      

      #let's get the path points and compute poly coef
      pts = np.array(lat_plan.lateralPlan.dPathPoints)
      x = np.arange(0, len(pts))
      order = 3
      coefs = np.polyfit(x, pts, order)
      # IC shows the path 2x scaled
      f = IC_LANE_SCALE
      suppress_x_coord = True
      f2 = f * f
      f3 = f2 * f
      CS.curvC0 = -clip(coefs[3], -3.5, 3.5)
      CS.curvC1 = -clip(coefs[2] * f * (0 if suppress_x_coord else 1), -0.2, 0.2)  
      CS.curvC2 = -clip(coefs[1] * f2, -0.0025, 0.0025)
      CS.curvC3 = -clip(coefs[0] * f3, -0.00003, 0.00003)  
      CS.laneWidth = lat_plan.lateralPlan.laneWidth
      CS.lProb = lat_plan.lateralPlan.lProb
      CS.rProb = lat_plan.lateralPlan.rProb
      if CS.lProb > 0.45:
        CS.lLine = 1
      else:
        CS.lLine = 0
      if CS.rProb > 0.45:
        CS.rLine = 1
      else:
        CS.rLine = 0
    
    # TODO: additional lanes to show on IC
    
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
    CS.prev_alca_pre_engage = CS.alca_pre_engage

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
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, lkas_enabled  and not human_control and not CS.out.cruiseState.standstill, CAN_EPAS[self.CP.carFingerprint], 1))

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

    if (not enabled) and self.CP.openpilotLongitudinalControl and (frame %2 == 0) and (self.CP.carFingerprint in [CAR.AP1_MODELS,CAR.AP2_MODELS]):
      #send this values so we can enable at 0 km/h
      tesla_accel_limits = [-1.4000000000000004,1.8000000000000007]
      tesla_jerk_limits = [-0.46000000000000085,0.47600000000000003]
      if self.CP.carFingerprint == CAR.AP2_MODELS:
        can_sends.append(self.tesla_can.create_ap2_long_control(350.0/3.6, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))
      if self.CP.carFingerprint == CAR.AP1_MODELS:
        can_sends.append(self.tesla_can.create_ap1_long_control(350.0/3.6, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

    #send messages for IC intergration
    CS.DAS_025_steeringOverride = 1 if human_control else 0
    CS.DAS_206_apUnavailable = 1 if human_control else 0
    warnings = CS.DAS_gas_to_resume + \
              CS.DAS_025_steeringOverride + \
              CS.DAS_202_noisyEnvironment + \
              CS.DAS_206_apUnavailable + \
              CS.DAS_207_lkasUnavailable + \
              CS.DAS_208_rackDetected + \
              CS.DAS_211_accNoSeatBelt + \
              CS.DAS_216_driverOverriding + \
              CS.DAS_219_lcTempUnavailableSpeed + \
              CS.DAS_220_lcTempUnavailableRoad + \
              CS.DAS_221_lcAborting + \
              CS.DAS_222_accCameraBlind + \
              CS.stopSignWarning + \
              CS.stopLightWarning + \
              CS.DAS_canErrors + \
              CS.DAS_notInDrive
    if (warnings > 0) and (self.IC_integration_warning_counter == 0):
      self.IC_integration_warning_counter == 200 # alert for 2 sconds
    if self.IC_integration_warning_counter == 0 or not enabled:
      # when zero reset all warnings
      CS.DAS_gas_to_resume = 0
      CS.DAS_025_steeringOverride = 0 #use for manual steer?
      CS.DAS_202_noisyEnvironment = 0 #use for planner error?
      CS.DAS_206_apUnavailable = 0 #Ap disabled from CID
      CS.DAS_207_lkasUnavailable = 0 #use for manual not in drive?
      CS.DAS_208_rackDetected = 0 #use for low battery?
      CS.DAS_211_accNoSeatBelt = 0
      CS.DAS_216_driverOverriding = 0
      CS.DAS_219_lcTempUnavailableSpeed = 0
      CS.DAS_220_lcTempUnavailableRoad = 0
      CS.DAS_221_lcAborting = 0
      CS.DAS_222_accCameraBlind = 0 #we will see what we can use this for
      CS.stopSignWarning = 0
      CS.stopLightWarning = 0
      CS.DAS_canErrors = 0
      CS.DAS_notInDrive = 0

    if (enabled or self.IC_previous_enabled or self.CP.carFingerprint == CAR.PREAP_MODELS) and (self.IC_integration_counter % 10 == 0):
      can_sends.append(self.tesla_can.create_lane_message(CS.laneWidth, 1 if CS.alca_engaged else CS.rLine, 1 if CS.alca_engaged else CS.lLine, 
          50, CS.curvC0, CS.curvC1, CS.curvC2, CS.curvC3, 
          CAN_CHASSIS[self.CP.carFingerprint], 1))

      # send DAS_bodyControls
      can_sends.append(self.tesla_can.create_body_controls_message(CS.msg_das_body_controls,
          CS.alca_direction, 1 if CS.needs_hazard else 0 , CAN_CHASSIS[self.CP.carFingerprint], 1))

      # send DAS_warningMatrix0 at 1Hz
      if self.IC_integration_counter == 10:
        can_sends.append(self.tesla_can.create_das_warningMatrix0(CS.DAS_canErrors, CS.DAS_025_steeringOverride, CS.DAS_notInDrive, CAN_CHASSIS[self.CP.carFingerprint]))
      
      # send DAS_warningMatrix1 at 1Hz
      if self.IC_integration_counter == 20:
        can_sends.append(self.tesla_can.create_das_warningMatrix1(CAN_CHASSIS[self.CP.carFingerprint]))

      # send DAS_warningMatrix3 at 1Hz
      if self.IC_integration_counter == 30:
        can_sends.append(self.tesla_can.create_das_warningMatrix3 (CS.DAS_gas_to_resume, CS.DAS_211_accNoSeatBelt, CS.DAS_202_noisyEnvironment, CS.DAS_206_apUnavailable, CS.DAS_207_lkasUnavailable,
          CS.DAS_219_lcTempUnavailableSpeed, CS.DAS_220_lcTempUnavailableRoad, CS.DAS_221_lcAborting, CS.DAS_222_accCameraBlind,
          CS.DAS_208_rackDetected, CS.DAS_216_driverOverriding, CS.stopSignWarning, CS.stopLightWarning, CAN_CHASSIS[self.CP.carFingerprint]))
      
      # send DAS_status and DAS_status2 at 2Hz
      if self.IC_integration_counter == 40 or self.IC_integration_counter == 90 or (self.IC_previous_enabled and not enabled ):
        DAS_ldwStatus = 1 if left_lane_depart or right_lane_depart else 0
        DAS_hands_on_state = 2
        #steering required is also used by ALCA
        if (hud_alert == VisualAlert.steerRequired) and not (CS.alca_engaged or CS.alca_pre_engage):
          if audible_alert == AudibleAlert.none:
            DAS_hands_on_state = 3
          else:
            DAS_hands_on_state = 5
        DAS_collision_warning =  1 if hud_alert == VisualAlert.fcw else 0
        #alcaState 1 if nothing, 8+direction if enabled
        DAS_alca_state = 8 + CS.alca_direction if (CS.alca_pre_engage or CS.alca_engaged) and CS.alca_direction > 0 else 1
        #ap status 0-Disabled 1-Unavailable 2-Available 3-Active_nominal, 
        #          4-active_restricted 5-active_nav 8-aborting 9-aborted
        #          14-fault  15-SNA
        DAS_op_status = 5 if enabled else 2
        can_sends.append(self.tesla_can.create_das_status(CS.msg_autopilot_status,DAS_op_status, DAS_collision_warning,
          DAS_ldwStatus, DAS_hands_on_state, DAS_alca_state, 
          CS.DAS_fusedSpeedLimit, CAN_CHASSIS[self.CP.carFingerprint], 1))
        can_sends.append(self.tesla_can.create_das_status2(CS.msg_autopilot_status2,CS.out.cruiseState.speed * CV.MS_TO_MPH, 
          DAS_collision_warning, CAN_CHASSIS[self.CP.carFingerprint], 1))
      
      self.IC_previous_enabled = enabled

    return can_sends
