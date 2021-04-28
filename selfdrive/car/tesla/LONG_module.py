from selfdrive.car.tesla.values import CAR, CAN_CHASSIS, CAN_POWERTRAIN
from selfdrive.car.tesla.ACC_module import ACCController
from selfdrive.car.tesla.PCC_module import PCCController
from selfdrive.config import Conversions as CV

def _is_present(lead):
  return bool((not (lead is None)) and (lead.dRel > 0))

class LONGController:

    def __init__(self,CP,packer, tesla_can):
        self.CP = CP
        self.v_target = None
        self.lead_1 = None
        self.long_control_counter = 1
        self.tesla_can = tesla_can
        self.packer = packer
        if (CP.carFingerprint == CAR.PREAP_MODELS):
            self.ACC = ACCController(self)
            self.PCC = PCCController(self,tesla_can)
            self.speed_limit_ms = 0
            self.set_speed_limit_active = False
            self.speed_limit_offset_ms = 0
            self.adaptive_cruise = 0


    def update(self, enabled, CS, frame, actuators, cruise_cancel,pcm_speed,pcm_override, long_plan,radar_state):
        messages = []

        if frame % 100 == 0:
            self.speed_limit_offset_ms = CS.userSpeedLimitOffsetMS
        if frame % 10 == 0:
            self.speed_limit_ms = CS.speed_limit_ms
            self.set_speed_limit_active = True if self.speed_limit_ms > 0 else False


        #preAP ModelS
        if self.CP.carFingerprint == CAR.PREAP_MODELS:
            # update PCC module info
            pedal_can_sends = self.PCC.update_stat(CS, frame)
            messages.extend(pedal_can_sends)
            if self.PCC.pcc_available:
                self.ACC.enable_adaptive_cruise = False
            else:
                # Update ACC module info.
                self.ACC.update_stat(CS, True)
                self.PCC.enable_pedal_cruise = False
            # update CS.v_cruise_pcm based on module selected.
            speed_uom_kph = 1.0
            if CS.speed_units == "MPH":
                speed_uom_kph = CV.KPH_TO_MPH
            if self.ACC.enable_adaptive_cruise:
                CS.acc_speed_kph = self.ACC.acc_speed_kph
            elif self.PCC.enable_pedal_cruise:
                CS.acc_speed_kph = self.PCC.pedal_speed_kph
            else:
                CS.acc_speed_kph = max(0.0, CS.out.vEgo * CV.MS_TO_KPH)
            CS.v_cruise_pcm = CS.acc_speed_kph * speed_uom_kph
            #determine cc_state
            CS.cc_state = 1
            if (self.PCC.pcc_available and self.PCC.enable_pedal_cruise) or (
                self.ACC.enable_adaptive_cruise
            ):
                CS.speed_control_enabled = 1
                CS.cc_state = 2
                if not self.ACC.adaptive:
                    CS.cc_state = 3
            self.adaptive_cruise = (
                1
                if (not self.PCC.pcc_available and self.ACC.adaptive)
                or self.PCC.pcc_available
                else 0
            )
            if (not self.PCC.pcc_available) and frame % 5 == 0:  # acc processed at 20Hz
                cruise_btn = self.ACC.update_acc(
                    enabled,
                    CS,
                    frame,
                    actuators,
                    pcm_speed,
                    self.speed_limit_ms * CV.MS_TO_KPH,
                    self.set_speed_limit_active,
                    self.speed_limit_offset_ms * CV.MS_TO_KPH,
                )
                if cruise_btn:
                    for counter in range(16):  # all possible message counters
                        messages.append(self.tesla_can.create_action_request(
                          msg_stw_actn_req=CS.msg_stw_actn_req,
                          cancel=False,
                          bus=CAN_CHASSIS[self.CP.carFingerprint],
                          counter=counter))
            apply_accel = 0.0
            if self.PCC.pcc_available and frame % 5 == 0:  # pedal processed at 20Hz
                pedalcan = 2
                apply_accel, accel_needed, accel_idx = self.PCC.update_pdl(
                    enabled,
                    CS,
                    frame,
                    actuators,
                    pcm_speed,
                    pcm_override,
                    self.speed_limit_ms,
                    self.set_speed_limit_active,
                    self.speed_limit_offset_ms,
                    CS.alca_engaged,
                    radar_state
                )
                messages.append(
                    self.tesla_can.create_pedal_command_msg(
                        apply_accel, int(accel_needed), accel_idx, pedalcan
                    )
                )

            #TODO: update message sent in HUD


        #AP ModelS with OP Long and enabled
        elif enabled and self.CP.openpilotLongitudinalControl and (frame %2 == 0) and (self.CP.carFingerprint in [CAR.AP1_MODELS,CAR.AP2_MODELS]):
            #we use the same logic from planner here to get the speed

            if radar_state is not None:
                self.lead_1 = radar_state.radarState.leadOne
            if long_plan is not None:
                self.v_target = long_plan.longitudinalPlan.vTargetFuture # to try vs vTarget
                self.a_target = abs(long_plan.longitudinalPlan.aTarget) # to try vs aTarget
            if self.v_target is None:
                self.v_target = CS.out.vEgo
                self.a_target = 1

            #following = False
            #TODO: see what works best for these
            tesla_accel_limits = [-2*self.a_target,self.a_target]
            tesla_jerk_limits = [-4*self.a_target,2*self.a_target]
            #if _is_present(self.lead_1):
            #  following = self.lead_1.status and self.lead_1.dRel < 45.0 and self.lead_1.vLeadK > CS.out.vEgo and self.lead_1.aLeadK > 0.0

            #we now create the DAS_control for AP1 or DAS_longControl for AP2
            if self.CP.carFingerprint == CAR.AP2_MODELS:
                messages.append(self.tesla_can.create_ap2_long_control(self.v_target, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))
            if self.CP.carFingerprint == CAR.AP1_MODELS:
                messages.append(self.tesla_can.create_ap1_long_control(self.v_target, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

        #AP ModelS with OP Long and not enabled
        elif (not enabled) and self.CP.openpilotLongitudinalControl and (frame %2 == 0) and (self.CP.carFingerprint in [CAR.AP1_MODELS,CAR.AP2_MODELS]):
            #send this values so we can enable at 0 km/h
            tesla_accel_limits = [-1.4000000000000004,1.8000000000000007]
            tesla_jerk_limits = [-0.46000000000000085,0.47600000000000003]
            if self.CP.carFingerprint == CAR.AP2_MODELS:
                messages.append(self.tesla_can.create_ap2_long_control(350.0/3.6, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))
            if self.CP.carFingerprint == CAR.AP1_MODELS:
                messages.append(self.tesla_can.create_ap1_long_control(350.0/3.6, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

        return messages

