from selfdrive.car.tesla.values import CarControllerParams, CAR, CAN_CHASSIS, CAN_POWERTRAIN, CruiseState
from selfdrive.car.tesla.ACC_module import ACCController
from selfdrive.car.tesla.PCC_module import PCCController
from selfdrive.config import Conversions as CV
from selfdrive.car.modules.CFG_module import load_bool_param,load_float_param
from cereal import car
from common.numpy_fast import interp

ACCEL_MULTIPLIERS_BP =     [0.0, 5.0, 10.0, 30.0]
ACCEL_MULT_SPEED_V  =      [1.5, 1.3,  1.2,  1.0]
ACCEL_MULT_SPEED_DELTA_V = [1.0, 1.01, 1.05,  1.1]
ACCEL_MULT_ACCEL_PERC_V  = [1.0, 1.0,  1.05,  1.1]

def _is_present(lead):
  return bool((not (lead is None)) and (lead.dRel > 0))

def _get_accel_multiplier(speed,speed_target,accel,accel_target):
    mult = 1.0
    mult = mult * interp(speed,ACCEL_MULTIPLIERS_BP,ACCEL_MULT_SPEED_V)
    mult = mult * interp(abs(speed-speed_target),ACCEL_MULTIPLIERS_BP,ACCEL_MULT_SPEED_DELTA_V)
    #how much we're expecting to change?
    da_perc = 100. * abs(accel_target - accel)/abs(accel)
    mult = mult * interp(da_perc,ACCEL_MULTIPLIERS_BP,ACCEL_MULT_ACCEL_PERC_V)
    return mult

class LONGController: 

    def __init__(self,CP,packer, tesla_can, pedalcan):
        self.CP = CP
        self.v_target = None
        self.a_target = None
        self.j_target = None
        self.lead_1 = None
        self.long_control_counter = 1
        self.tesla_can = tesla_can
        self.packer = packer
        self.pedalcan = pedalcan
        self.has_ibooster_ecu = False
        self.apply_brake = 0.0
        self.speed_limit_offset_uom = 0
        self.speed_limit_offset_ms = 0.0
        self.adjustSpeedWithSpeedLimit = load_bool_param("TinklaAdjustAccWithSpeedLimit",True)
        self.useBrakeWipe = load_bool_param("TinklaUseBrakeWipe", False)
        self.madMax = load_bool_param("TinklaSpeedMadMax",False)
        if (CP.carFingerprint == CAR.PREAP_MODELS):
            self.ACC = ACCController(self)
            self.PCC = PCCController(self,tesla_can,pedalcan)
            self.speed_limit_ms = 0
            self.set_speed_limit_active = False
            self.speed_limit_offset_uom = load_float_param("TinklaSpeedLimitOffset",0.0)
            

    def update(self, enabled, CS, frame, actuators, cruise_cancel,pcm_speed,pcm_override, long_plan,radar_state):
        messages = []
        self.has_ibooster_ecu = CS.has_ibooster_ecu
        #update options 

        if frame % 10 == 0:
            self.speed_limit_ms = CS.speed_limit_ms
            self.set_speed_limit_active = self.adjustSpeedWithSpeedLimit if self.speed_limit_ms > 0 else False

        if frame % 100 == 0:
            if self.CP.carFingerprint != CAR.PREAP_MODELS:
                self.speed_limit_offset_ms = CS.userSpeedLimitOffsetMS
            else:
                if load_bool_param("TinklaSpeedLimitUseRelative",False):
                    self.speed_limit_offset_ms = self.speed_limit_offset_uom * self.speed_limit_ms / 100.0
                else:
                    if CS.speed_units == "KPH":
                        self.speed_limit_offset_ms = self.speed_limit_offset_uom* CV.KPH_TO_MS
                    elif CS.speed_units == "MPH":
                        self.speed_limit_offset_ms = self.speed_limit_offset_uom* CV.MPH_TO_MS
        

        #if not openpilot long control just exit
        if not self.CP.openpilotLongitudinalControl:
            return messages
        #preAP ModelS
        if self.CP.carFingerprint == CAR.PREAP_MODELS:
            # update PCC module info
            pedal_can_sends = self.PCC.update_stat(CS, frame)
            if len(pedal_can_sends) > 0:
                messages.extend(pedal_can_sends)
            if self.PCC.pcc_available:
                self.ACC.enable_adaptive_cruise = False
            else:
                # Update ACC module info.
                self.PCC.enable_pedal_cruise = False
                self.ACC.update_stat(CS, True)
            # update CS.v_cruise_pcm based on module selected.
            speed_uom_kph = 1.0
            # cruise state: 0 unavailable, 1 available, 2 enabled, 3 hold
            if CS.carNotInDrive:
                CS.cc_state = 0
            else:
                CS.cc_state = 1
            CS.speed_control_enabled = 0
            if enabled:
                if CS.speed_units == "MPH":
                    speed_uom_kph = CV.KPH_TO_MPH
                if self.ACC.enable_adaptive_cruise:
                    CS.acc_speed_kph = self.ACC.acc_speed_kph
                elif self.PCC.enable_pedal_cruise:
                    CS.acc_speed_kph = self.PCC.pedal_speed_kph
                else:
                    CS.acc_speed_kph = max(0.0, CS.out.vEgo * CV.MS_TO_KPH)
                CS.v_cruise_pcm = CS.acc_speed_kph * speed_uom_kph
                if (self.PCC.pcc_available and self.PCC.enable_pedal_cruise) or (
                    self.ACC.enable_adaptive_cruise
                ):
                    CS.speed_control_enabled = 1
                    CS.cc_state = 2
                    if not self.ACC.adaptive:
                        CS.cc_state = 3 #find other values we can use
            else:
                if CS.cruise_state == CruiseState.OVERRIDE:  # state #4
                    CS.cc_state = 3
            CS.adaptive_cruise = (
                1
                if (not self.PCC.pcc_available and self.ACC.adaptive)
                or self.PCC.pcc_available
                else 0
            )
            CS.pcc_available = self.PCC.pcc_available
           
            if long_plan and long_plan.longitudinalPlan:
                self.v_target = long_plan.longitudinalPlan.speeds[-1]
            if (not self.PCC.pcc_available) and frame % 20 == 0:  # acc processed at 5Hz
                cruise_btn = self.ACC.update_acc(
                    enabled,
                    CS,
                    frame,
                    actuators,
                    self.v_target,
                    self.speed_limit_ms * CV.MS_TO_KPH,
                    self.set_speed_limit_active,
                    self.speed_limit_offset_ms * CV.MS_TO_KPH,
                )
                if cruise_btn:
                    # insert the message first since it is racing against messages from the real stalk
                    stlk_counter = ((CS.msg_stw_actn_req['MC_STW_ACTN_RQ'] + 1) % 16)
                    messages.insert(0, self.tesla_can.create_action_request(
                         msg_stw_actn_req=CS.msg_stw_actn_req,
                         button_to_press=cruise_btn,
                         bus=CAN_CHASSIS[self.CP.carFingerprint],
                         counter=stlk_counter))
            apply_accel = 0.0
            if self.PCC.pcc_available and frame % 5 == 0:  # pedal processed at 20Hz
                v_target = 0
                if long_plan is not None:
                    v_target = long_plan.longitudinalPlan.speeds[0]
                self.apply_brake = 0.0
                apply_accel, self.apply_brake, accel_needed, accel_idx = self.PCC.update_pdl(
                    enabled,
                    CS,
                    frame,
                    actuators,
                    v_target,
                    pcm_override,
                    self.speed_limit_ms,
                    self.set_speed_limit_active,
                    self.speed_limit_offset_ms,
                    CS.alca_engaged,
                    radar_state
                )
                if (accel_needed > -1) and (accel_idx > -1):
                    messages.append(
                        self.tesla_can.create_pedal_command_msg(
                            apply_accel, int(accel_needed), accel_idx, self.pedalcan
                        )
                    )
                    
            if self.PCC.pcc_available:
                if self.has_ibooster_ecu:
                    messages.append(
                        self.tesla_can.create_ibst_command(
                            enabled, self.apply_brake, frame, CAN_CHASSIS[self.CP.carFingerprint]
                        )
                    )
                else:
                    # let's try to use brake wipe to slow down the car
                    # GTW_ESP1 is at 10Hz and we will spam at 100Hz
                    if self.apply_brake >= 0.1:
                        CS.gtw_esp1_bw_req = 2 #hard wipe
                        if self.useBrakeWipe:
                            CS.longCtrlEvent = car.CarEvent.EventName.brakeWipeHigh
                    elif self.apply_brake > 0.0:
                        CS.gtw_esp1_bw_req = 1 #soft wipe
                        if self.useBrakeWipe:
                            CS.longCtrlEvent = car.CarEvent.EventName.brakeWipeLow
                    else:
                        CS.gtw_esp1_bw_req = 0 #no wipe
                    if CS.gtw_esp1_bw_req > 0:
                        if CS.prev_gtw_esp1_bw_req == 0:
                           #first time BW request happens
                           CS.gtw_esp1_id = CS.gtw_esp1_last_sent_id
                        CS.gtw_esp1_id = (CS.gtw_esp1_id + 1) % 8
                        if (CS.gtw_esp1 is None) and self.useBrakeWipe:
                            CS.longCtrlEvent = car.CarEvent.EventName.brakeWipeNotAvailable
                        if (CS.gtw_esp1 is not None) and self.useBrakeWipe:
                            messages.insert(0, self.tesla_can.create_brake_wipe_request(
                                gtw_esp1_vals=CS.gtw_esp1,
                                bw_req=CS.gtw_esp1_bw_req,
                                bus=CAN_CHASSIS[self.CP.carFingerprint],
                                counter=CS.gtw_esp1_id))
                    CS.prev_gtw_esp1_bw_req = CS.gtw_esp1_bw_req

            #TODO: update message sent in HUD

        #AP ModelS with OP Long and enabled
        elif enabled and self.CP.openpilotLongitudinalControl and (frame %2 == 0) and (self.CP.carFingerprint in [CAR.AP1_MODELS,CAR.AP2_MODELS]):
            #we use the same logic from planner here to get the speed
            speed_uom_kph = 1.0
            # cruise state: 0 unavailable, 1 available, 2 enabled, 3 hold
            if CS.carNotInDrive:
                CS.cc_state = 0
            else:
                CS.cc_state = 1
            CS.speed_control_enabled = 0
            if CS.speed_units == "MPH":
                speed_uom_kph = CV.KPH_TO_MPH
            CS.acc_speed_kph = CS.cruise_speed * CV.MS_TO_KPH
            CS.v_cruise_pcm = CS.acc_speed_kph * speed_uom_kph
            CS.speed_control_enabled = 1
            CS.cc_state = 3 # was 2, we use HOLD to show it's OP for now
            CS.adaptive_cruise = 1
            if radar_state is not None:
                self.lead_1 = radar_state.radarState.leadOne
            if long_plan is not None:
                self.v_target = long_plan.longitudinalPlan.speeds[-1] # 0 or -1 to try vs actual vs vTarget
                self.a_target = abs(long_plan.longitudinalPlan.accels[-1]) #0 or -1 to try actual vs aTarget
                self.j_target = abs(long_plan.longitudinalPlan.jerks[-1]) # -1 to try actual vs jTarget
            if self.v_target is None:
                self.v_target = CS.out.vEgo
                self.a_target = 1
                self.j_target = 1

            #following = False
            #TODO: see what works best for these
            #tesla_accel_limits = [-self.a_target,self.a_target]
            #tesla_jerk_limits = [-self.j_target,self.j_target]
            #if _is_present(self.lead_1):
            #  following = self.lead_1.status and self.lead_1.dRel < 45.0 and self.lead_1.vLeadK > CS.out.vEgo and self.lead_1.aLeadK > 0.0
            target_accel = actuators.accel 
            if self.madMax:
                target_accel = target_accel * _get_accel_multiplier(CS.out.vEgo,self.v_target,actuators.accel,self.a_target)
            target_speed = max(CS.out.vEgo + (target_accel * CarControllerParams.ACCEL_TO_SPEED_MULTIPLIER), 0)
            target_speed = target_speed * CV.MS_TO_KPH
            max_accel = 0 if target_accel < 0 else target_accel
            min_accel = 0 if target_accel > 0 else target_accel
            tesla_jerk_limits = [CarControllerParams.JERK_LIMIT_MIN,CarControllerParams.JERK_LIMIT_MAX]
            tesla_accel_limits = [min_accel,max_accel]
            if self.madMax:
                tesla_jerk_limits = [min_accel/2,max_accel/2]
            #we now create the DAS_control for AP1 or DAS_longControl for AP2
            if self.CP.carFingerprint == CAR.AP2_MODELS:
                messages.append(self.tesla_can.create_ap2_long_control(target_speed, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))
            if self.CP.carFingerprint == CAR.AP1_MODELS:
                messages.append(self.tesla_can.create_ap1_long_control(not CS.carNotInDrive, False, enabled ,target_speed, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

        #AP ModelS with OP Long and not enabled
        elif (not enabled) and (not CS.autopilot_enabled) and (not CS.autopilot_was_enabled) and self.CP.openpilotLongitudinalControl and (frame %2 == 0) and (self.CP.carFingerprint in [CAR.AP1_MODELS,CAR.AP2_MODELS]):
            if CS.carNotInDrive:
                CS.cc_state = 0
            else:
                CS.cc_state = 1
            #send this values so we can enable at 0 km/h
            tesla_accel_limits = [-1.4000000000000004,1.8000000000000007]
            tesla_jerk_limits = [-0.46000000000000085,0.47600000000000003]
            if self.CP.carFingerprint == CAR.AP2_MODELS:
                messages.append(self.tesla_can.create_ap2_long_control(350.0, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))
            if self.CP.carFingerprint == CAR.AP1_MODELS:
                messages.append(self.tesla_can.create_ap1_long_control(not CS.carNotInDrive, False, False , 350.0, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

        return messages

