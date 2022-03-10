from common.numpy_fast import clip
from selfdrive.car.tesla.values import CAR, CAN_CHASSIS
from cereal import car
from selfdrive.config import Conversions as CV
import numpy as np

IC_LANE_SCALE = 0.5

VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert

class HUDController: 

    def __init__(self, CP, packer, tesla_can):
        self.CP = CP
        self.packer = packer
        self.tesla_can = tesla_can
        
        self.IC_integration_counter = 0
        self.IC_integration_warning_counter = 0
        self.IC_previous_enabled = False
        self.radarVin_idx = 0
        self.leftLaneQuality = 0
        self.rightLaneQuality = 0
        

    # to show lead car on IC
    def showLeadCarOnICCanMessage(self, leadsData, curv0):
        lead_1 = leadsData.leadOne
        lead_2 = leadsData.leadTwo
        if (lead_1 is not None) and lead_1.status:
            self.leadDx = clip(lead_1.dRel,0,126)
            self.leadDy = clip(curv0 - lead_1.yRel,-22.05,22.4)
            self.leadId = 1
            self.leadClass = 2
            self.leadVx = clip(int(lead_1.vRel),-30,26)
        else:
            self.leadDx = 0
            self.leadDy = 0.0
            self.leadClass = 0
            self.leadId = 0
            self.leadVx = 0
        if (lead_2 is not None) and lead_2.status:
            self.lead2Dx = clip(lead_2.dRel,0,126)
            self.lead2Dy = clip(curv0 - lead_2.yRel,-22.05,22.4)
            self.lead2Id = 2
            self.lead2Class = 2
            self.lead2Vx = clip(int(lead_2.vRel),-30,26)
        else:
            self.lead2Dx = 0
            self.lead2Dy = 0.0
            self.lead2Class = 0
            self.lead2Id = 0
            self.lead2Vx = 0
        return self.tesla_can.create_lead_car_object_message(
                0, #lead vehicle
                self.leadClass,
                self.leadId,
                0, #relevant1
                self.leadDx,
                self.leadVx,
                self.leadDy,
                self.lead2Class,
                self.lead2Id,
                0, #relevant2
                self.lead2Dx,
                self.lead2Vx,
                self.lead2Dy,
                CAN_CHASSIS[self.CP.carFingerprint],
            )

    def update(self, enabled, CS, frame, actuators, cruise_cancel, hud_alert, audible_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart,human_control,radar_state,lat_plan,apply_angle,model_data):
        # TODO: additional lanes to show on IC
        self.IC_integration_counter = ((self.IC_integration_counter + 2) % 100)

        if self.IC_integration_warning_counter > 0:
            self.IC_integration_warning_counter = self.IC_integration_warning_counter - 1
        messages = []

        if lat_plan is not None:
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
        if model_data is not None:
            self.leftLaneQuality = 1 if model_data.modelV2.laneLineProbs[0] > 0.25 else 0
            self.rightLaneQuality = 1 if model_data.modelV2.laneLineProbs[3] > 0.25 else 0

        #send messages for IC intergration
        #CS.DAS_206_apUnavailable = 1 if enabled and human_control else 0
        #CS.DAS_220_lcTempUnavailableRoad = 1 if enabled and human_control else 0
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
            self.IC_integration_warning_counter = 200 # alert for 2 sconds
        if (self.IC_integration_warning_counter == 0) or (not enabled):
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

            # send DAS_bodyControls
            if (self.IC_integration_counter in [20,70]) or (self.IC_previous_enabled and not enabled):
                messages.append(self.tesla_can.create_body_controls_message(
                CS.alca_direction, 1 if CS.needs_hazard else 0 , CAN_CHASSIS[self.CP.carFingerprint], 1))
            #if no ic integration we stop here
            if not CS.enableICIntegration:
                return messages

            messages.append(self.tesla_can.create_lane_message(CS.laneWidth, 1 if CS.alca_engaged else CS.rLine, 1 if CS.alca_engaged else CS.lLine, 
                50, CS.curvC0, CS.curvC1, CS.curvC2, CS.curvC3, self.leftLaneQuality, self.rightLaneQuality,
                CAN_CHASSIS[self.CP.carFingerprint], 1))

            if radar_state is not None:
                leadsData = radar_state.radarState
                if leadsData is not None:
                   #pass
                   messages.append(self.showLeadCarOnICCanMessage(leadsData=leadsData,curv0=CS.curvC0))
            
            # send DAS_warningMatrix0 at 1Hz
            if (self.IC_integration_counter == 10) or (self.IC_previous_enabled and not enabled):
                messages.append(self.tesla_can.create_das_warningMatrix0(CS.DAS_canErrors, CS.DAS_025_steeringOverride, CS.DAS_notInDrive, CAN_CHASSIS[self.CP.carFingerprint]))
            
            # send DAS_warningMatrix1 at 1Hz
            if (self.IC_integration_counter == 20) or (self.IC_previous_enabled and not enabled):
                messages.append(self.tesla_can.create_das_warningMatrix1(CAN_CHASSIS[self.CP.carFingerprint]))

            # send DAS_warningMatrix3 at 1Hz
            if (self.IC_integration_counter == 30) or (self.IC_previous_enabled and not enabled):
                messages.append(self.tesla_can.create_das_warningMatrix3 (CS.DAS_gas_to_resume, CS.DAS_211_accNoSeatBelt, CS.DAS_202_noisyEnvironment, CS.DAS_206_apUnavailable, CS.DAS_207_lkasUnavailable,
                    CS.DAS_219_lcTempUnavailableSpeed, CS.DAS_220_lcTempUnavailableRoad, CS.DAS_221_lcAborting, CS.DAS_222_accCameraBlind,
                    CS.DAS_208_rackDetected, CS.DAS_216_driverOverriding, CS.stopSignWarning, CS.stopLightWarning, CAN_CHASSIS[self.CP.carFingerprint]))
            
            # send DAS_status and DAS_status2 at 2Hz
            DAS_ldwStatus = 1 if left_lane_depart or right_lane_depart else 0
            DAS_hands_on_state = 2
            #steering required is also used by ALCA
            if (hud_alert == VisualAlert.steerRequired) and not (CS.alca_engaged or CS.alca_pre_engage):
                if audible_alert == AudibleAlert.none:
                    DAS_hands_on_state = 3
                else:
                    DAS_hands_on_state = 5
            #if manual steering overright we will flash the light at the top of IC
            if enabled and human_control:
                DAS_hands_on_state = 5
            DAS_collision_warning =  1 if hud_alert == VisualAlert.fcw else 0
            #alcaState 1 if nothing, 8+direction if enabled
            DAS_alca_state = 8 + CS.alca_direction if (CS.alca_pre_engage or CS.alca_engaged) and CS.alca_direction > 0 else 1
            #ap status 0-Disabled 1-Unavailable 2-Available 3-Active_nominal, 
            #          4-active_restricted 5-active_nav 8-aborting 9-aborted
            #          14-fault  15-SNA
            DAS_op_status = 5 if enabled else 2
            if (self.IC_integration_counter %20 == 0) or (self.IC_previous_enabled and not enabled ):
                messages.append(self.tesla_can.create_das_status(DAS_op_status, DAS_collision_warning,
                    DAS_ldwStatus, DAS_hands_on_state, DAS_alca_state, 
                    CS.out.leftBlindspot, CS.out.rightBlindspot,
                    CS.DAS_fusedSpeedLimit, CAN_CHASSIS[self.CP.carFingerprint], 1))
                messages.append(self.tesla_can.create_das_status2(CS.out.cruiseState.speed * CV.MS_TO_MPH, 
                    DAS_collision_warning, CAN_CHASSIS[self.CP.carFingerprint], 1))
            self.IC_previous_enabled = enabled

            #send message for TB if preAP
            if (self.CP.carFingerprint == CAR.PREAP_MODELS) and (self.IC_integration_counter %10 == 0):
                speed_uom_kph = 1.0
                if CS.speed_units == "MPH":
                    speed_uom_kph = CV.KPH_TO_MPH
                v_cruise_pcm = max(0.0, CS.out.vEgo * CV.MS_TO_KPH) * speed_uom_kph
                if CS.cruiseEnabled:
                    v_cruise_pcm = max(0.0, CS.out.cruiseState.speed * CV.MS_TO_KPH) * speed_uom_kph
                DAS_control_speed = v_cruise_pcm
                if CS.carNotInDrive:
                    DAS_control_speed = 350.0/3.6
                messages.append(
                    self.tesla_can.create_ap1_long_control(
                        not CS.carNotInDrive, 
                        not CS.adaptive_cruise,
                        CS.cc_state > 1,
                        DAS_control_speed,
                        [-1.4000000000000004,1.8000000000000007],
                        [-0.46000000000000085,0.47600000000000003],
                        CAN_CHASSIS[self.CP.carFingerprint], 
                        1
                    )
                )
                messages.append(
                    self.tesla_can.create_fake_DAS_msg(
                        CS.speed_control_enabled,
                        CS.DAS_216_driverOverriding, 
                        CS.DAS_206_apUnavailable,
                        DAS_collision_warning,
                        DAS_op_status, 
                        max(0.0, CS.out.cruiseState.speed * CV.MS_TO_KPH),#
                        CS.tap_direction,
                        DAS_collision_warning,
                        CS.adaptive_cruise,
                        DAS_hands_on_state,
                        CS.cc_state,
                        1 if CS.pcc_available else 0, 
                        DAS_alca_state,
                        v_cruise_pcm,
                        CS.DAS_fusedSpeedLimit,
                        apply_angle,
                        1 if enabled else 0,
                        0, #park_brake_request
                        CAN_CHASSIS[self.CP.carFingerprint],
                    )
                )

        return messages
