from selfdrive.car.tesla.values import CruiseButtons, CarControllerParams, CAR, CAN_CHASSIS, CAN_POWERTRAIN, CruiseState, TESLA_MAX_ACCEL, TESLA_MIN_ACCEL
from selfdrive.car.tesla.ACC_module import ACCController
from selfdrive.car.tesla.PCC_module import PCCController
from selfdrive.config import Conversions as CV
from selfdrive.car.modules.CFG_module import load_bool_param,load_float_param
from common.numpy_fast import interp,clip
from selfdrive.car.tesla.speed_utils.fleet_speed import FleetSpeed

ACCEL_MULTIPLIERS_BP =     [0.0, 5.0, 10.0, 30.0]
ACCEL_MULT_SPEED_V  =      [1.5, 1.3,  1.2,  1.0]
ACCEL_MULT_SPEED_DELTA_V = [1.0, 1.01, 1.05,  1.1]
ACCEL_MULT_ACCEL_PERC_V  = [1.0, 1.0,  1.05,  1.1]

FLEET_SPEED_ACCEL = -0.5 # m/s2 how fast to reduce speed to match fleet, always negative 
BRAKE_FACTOR_BP = [18., 28.]
BRAKE_FACTOR_V = [1.15, 1.45]
BRAKE_FACTOR = load_float_param("TinklaBrakeFactor",1.0)

def _is_present(lead):
  return bool((not (lead is None)) and (lead.dRel > 0))

def _get_accel_multiplier(speed,speed_target,accel):
    mult = 1.0
    #only do it for positive acceleration
    if accel <= 0:
        return mult
    mult = mult * interp(speed,ACCEL_MULTIPLIERS_BP,ACCEL_MULT_SPEED_V)
    mult = mult * interp(abs(speed-speed_target),ACCEL_MULTIPLIERS_BP,ACCEL_MULT_SPEED_DELTA_V)
    return mult

class LONGController: 

    def __init__(self,CP,packer, tesla_can, pedalcan):
        self.CP = CP
        self.v_target = None
        self.a_target = 0.
        self.j_target = 0.
        self.lead_1 = None
        self.long_control_counter = 1
        self.tesla_can = tesla_can
        self.packer = packer
        self.pedalcan = pedalcan
        self.has_ibooster_ecu = False
        self.ibooster_idx = 0
        self.apply_brake = 0.0
        self.speed_limit_offset_uom = load_float_param("TinklaSpeedLimitOffset",0.0)
        self.speed_limit_offset_ms = 0.0
        self.adjustSpeedWithSpeedLimit = load_bool_param("TinklaAdjustAccWithSpeedLimit",True)
        self.adjustSpeedRelative = load_bool_param("TinklaSpeedLimitUseRelative",False)
        self.useLongControlData = load_bool_param("TinklaUseLongControlData",False)
        self.autopilot_disabled = load_bool_param("TinklaAutopilotDisabled",False)
        average_speed_over_x_suggestions = 5  # 1 second @ 5Hz
        self.speed_limit_uom = 0.
        self.prev_speed_limit_uom = 0.
        self.fleet_speed = FleetSpeed(average_speed_over_x_suggestions)
        self.fleet_speed_ms = 0.
        self.prev_enabled = False
        self.speed_limit_ms = 0.
        self.prev_speed_limit_ms = 0.
        self.prev_speed_limit_ms_das = 0.
        self.ap1_adjusting_speed = False
        self.ap1_speed_target = 0.
        self.set_speed_limit_active = False
        self.longPlan = None
        if (CP.carFingerprint == CAR.PREAP_MODELS) or self.autopilot_disabled:
            self.ACC = ACCController(self)
            self.PCC = PCCController(self,tesla_can,pedalcan,CP )
            
            
            

    def update(self, enabled, CS, frame, actuators, cruise_cancel,pcm_speed,pcm_override, long_plan,radar_state):
        messages = []
        self.has_ibooster_ecu = CS.has_ibooster_ecu
        my_accel = actuators.accel
        if my_accel < 0:
            my_accel = my_accel * BRAKE_FACTOR 
        #update options 
        
        target_accel = clip(my_accel, TESLA_MIN_ACCEL,TESLA_MAX_ACCEL)
        if self.CP.carFingerprint == CAR.PREAP_MODELS or CS.autopilot_disabled:
            target_accel = clip(my_accel * interp(CS.out.vEgo, BRAKE_FACTOR_BP, BRAKE_FACTOR_V ), TESLA_MIN_ACCEL,TESLA_MAX_ACCEL)

        max_accel = 0 if target_accel < 0 else target_accel
        min_accel = 0 if target_accel > 0 else target_accel

        max_jerk = CarControllerParams.JERK_LIMIT_MAX
        min_jerk = CarControllerParams.JERK_LIMIT_MIN

        tesla_jerk_limits = [min_jerk,max_jerk]
        tesla_accel_limits = [min_accel,max_accel]

        if enabled and not self.prev_enabled:
            self.fleet_speed.reset_averager()
        

        if frame % 10 == 0:
            self.speed_limit_ms = CS.speed_limit_ms
            if self.speed_limit_ms != self.prev_speed_limit_ms:
                self.fleet_speed.reset_averager() #reset fleet averager on speed limit changes
                self.prev_speed_limit_ms = self.speed_limit_ms
            self.set_speed_limit_active = self.adjustSpeedWithSpeedLimit if self.speed_limit_ms > 0 else False

        if frame % 100 == 0:
            if self.CP.carFingerprint != CAR.PREAP_MODELS:
                self.speed_limit_offset_uom = CS.userSpeedLimitOffsetMS
            if self.adjustSpeedRelative:
                if self.speed_limit_ms > 0:
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
        if self.CP.carFingerprint == CAR.PREAP_MODELS or CS.autopilot_disabled:
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
            CS.adaptive_cruise_enabled = (
                (self.ACC.adaptive and self.ACC.enable_adaptive_cruise and not self.PCC.pcc_available)
                or
                (self.PCC.enable_pedal_cruise)
            )
            CS.pcc_available = self.PCC.pcc_available
            CS.pcc_enabled = self.PCC.enable_pedal_cruise
           
            if long_plan and long_plan.longitudinalPlan:
                self.longPlan = long_plan.longitudinalPlan
                self.v_target = self.longPlan.speeds[-1]

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
            if self.PCC.pcc_available and frame % 1 == 0:  # pedal processed at 100Hz, we get speed at 50Hz from ESP_B
                #following = False
                #TODO: see what works best for these
                if self.longPlan:
                    self.v_target = self.longPlan.speeds[0]
                    self.a_target = self.longPlan.accels[0]
    
                if self.v_target is None:
                    self.v_target = CS.out.vEgo
                target_speed = max(self.v_target, 0)                    

                apply_accel, self.apply_brake, accel_needed, accel_idx = self.PCC.update_pdl(
                    enabled,
                    CS,
                    frame,
                    actuators,
                    target_speed,
                    target_accel,
                    self.a_target,
                    pcm_override,
                    self.speed_limit_ms,
                    self.set_speed_limit_active,
                    self.speed_limit_offset_ms,
                    CS.alca_engaged,
                    radar_state
                )
                #send pedal commands at 50Hz
                if (accel_needed > -1) and (accel_idx > -1) and frame % 2 == 0:
                    messages.append(
                        self.tesla_can.create_pedal_command_msg(
                            apply_accel, int(accel_needed), accel_idx, self.pedalcan
                        )
                    )
                    self.PCC.pedal_idx = (self.PCC.pedal_idx + 1) % 16
                #send ibooster commands at 10Hz    
                if self.has_ibooster_ecu  and frame % 10 == 0:
                    messages.append(
                        self.tesla_can.create_ibst_command(
                            enabled, 15 * self.apply_brake, self.ibooster_idx, CAN_CHASSIS[self.CP.carFingerprint]
                        )
                    )
                    self.ibooster_idx = (self.ibooster_idx + 1) % 16
                
                if self.autopilot_disabled:
                    target_speed = target_speed * CV.MS_TO_KPH
                    if enabled:
                        if self.CP.carFingerprint == CAR.AP1_MODELS:
                            messages.append(self.tesla_can.create_ap1_long_control(not CS.carNotInDrive, False, CS.pcc_enabled ,target_speed, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))
                    else:
                        if CS.carNotInDrive:
                            CS.cc_state = 0
                        else:
                            CS.cc_state = 1
                        #send this values so we can enable at 0 km/h
                        tesla_accel_limits = [-1.4000000000000004,1.8000000000000007]
                        tesla_jerk_limits = [-0.46000000000000085,0.47600000000000003]
                        if self.CP.carFingerprint == CAR.AP1_MODELS:
                            messages.append(self.tesla_can.create_ap1_long_control(not CS.carNotInDrive, False, False , 0, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

                
        #AP ModelS with OP Long and enabled
        elif enabled and self.CP.openpilotLongitudinalControl and (frame %1 == 0) and (self.CP.carFingerprint in [CAR.AP1_MODELS,CAR.AP2_MODELS]):
            #we use the same logic from planner here to get the speed
            speed_uom_kph = 1.0
            # cruise state: 0 unavailable, 1 available, 2 enabled, 3 hold
            if CS.carNotInDrive:
                CS.cc_state = 0
            else:
                CS.cc_state = 1
            if CS.speed_units == "MPH":
                speed_uom_kph = CV.KPH_TO_MPH
            CS.acc_speed_kph = CS.out.cruiseState.speed * CV.MS_TO_KPH
            acc_speed_uom_int = int(round(CS.acc_speed_kph * speed_uom_kph + 0.01))
            if self.set_speed_limit_active and self.speed_limit_ms > 0:
                self.speed_limit_uom = int(round((self.speed_limit_ms + self.speed_limit_offset_ms) * CV.MS_TO_KPH * speed_uom_kph + 0.01))
            if ( self.set_speed_limit_active and
            ((
                self.prev_speed_limit_uom != self.speed_limit_uom
                and self.speed_limit_ms > 0
                and acc_speed_uom_int != self.speed_limit_uom
            ) or (
                enabled and 
                not self.prev_enabled
                and self.speed_limit_ms > 0
            ) or (
                CS.speed_limit_ms_das != self.prev_speed_limit_ms_das
                and CS.speed_limit_ms_das > 0
                and self.speed_limit_ms > 0
            ))):
                self.ap1_adjusting_speed = True
                self.ap1_speed_target = self.speed_limit_uom
            self.prev_speed_limit_uom = self.speed_limit_uom
            self.prev_speed_limit_ms_das = CS.speed_limit_ms_das
            if self.ap1_adjusting_speed and acc_speed_uom_int == self.ap1_speed_target:
                self.ap1_adjusting_speed = False
                self.ap1_speed_target = 0
            if self.ap1_adjusting_speed  and frame % 33 == 0:
                #adjust speed at 5Hz
                speed_offset_uom = self.ap1_speed_target - acc_speed_uom_int
                
                button_to_press = None
                if speed_offset_uom <= -5:
                    button_to_press = CruiseButtons.DECEL_2ND
                elif speed_offset_uom <= -1:
                    button_to_press = CruiseButtons.DECEL_SET
                elif speed_offset_uom >= 5:
                    button_to_press = CruiseButtons.RES_ACCEL_2ND
                elif speed_offset_uom >= 1:
                    button_to_press = CruiseButtons.RES_ACCEL

                if button_to_press:
                    # insert the message first since it is racing against messages from the real stalk
                    stlk_counter = ((CS.msg_stw_actn_req['MC_STW_ACTN_RQ'] + 1) % 16)
                    messages.append(self.tesla_can.create_action_request(
                         msg_stw_actn_req=CS.msg_stw_actn_req,
                         button_to_press=button_to_press,
                         bus=CAN_CHASSIS[self.CP.carFingerprint],
                         counter=stlk_counter))
            CS.v_cruise_pcm = CS.acc_speed_kph * speed_uom_kph
            CS.speed_control_enabled = 1
            CS.cc_state = 3 # was 2, we use HOLD to show it's OP for now
            CS.adaptive_cruise = 1
            if radar_state is not None:
                self.lead_1 = radar_state.radarState.leadOne
            if long_plan is not None:
                self.v_target = long_plan.longitudinalPlan.speeds[-1] # 0 or -1 to try vs actual vs vTarget
                self.a_target = long_plan.longitudinalPlan.accels[-1] #0 or -1 to try actual vs aTarget
                self.j_target = long_plan.longitudinalPlan.jerks[-1] # 0 or  -1 to try actual vs jTarget
            if self.v_target is None:
                self.v_target = CS.out.vEgo
                self.a_target = 0
                self.j_target = 8

            #following = False
            target_speed = max(CS.out.vEgo + (target_accel * CarControllerParams.ACCEL_TO_SPEED_MULTIPLIER), 0)

            #if accel pedal pressed send 0 for target_accel
            if CS.realPedalValue > 0 and CS.enableHAO:
                target_accel = 0.

            if self.useLongControlData:
                if frame % 20 == 0: #update fleet speed info at 5 Hz
                    self.fleet_speed_ms = self.fleet_speed.adjust(
                            CS, CS.out.cruiseState.speed, frame
                        )
                if self.fleet_speed_ms < target_speed and self.fleet_speed_ms > 0:
                    target_accel = min(target_accel,FLEET_SPEED_ACCEL)
                    target_speed = min(target_speed,self.fleet_speed_ms,CS.out.cruiseState.speed)

            target_speed = target_speed * CV.MS_TO_KPH
            
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
                messages.append(self.tesla_can.create_ap1_long_control(not CS.carNotInDrive, False, False , 0, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

        self.prev_enabled = enabled
        return messages

