from selfdrive.car.tesla.values import CAN_POWERTRAIN
#from selfdrive.config import Conversions as CV

def _is_present(lead):
  return bool((not (lead is None)) and (lead.dRel > 0))

class LONGController:
    
    def __init___(self,CP,packer, tesla_can):
        self.CP = CP
        self.v_target = None
        self.lead_1 = None
        self.long_control_counter = 1
        self.tesla_can = tesla_can
        self.packer = packer


    def update(self, enabled, CS, frame, actuators, cruise_cancel,long_plan,radar_state):
        messages = []
        if enabled and self.CP.openpilotLongitudinalControl and (frame %2 == 0):
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

        if (not enabled) and self.CP.openpilotLongitudinalControl and (frame %2 == 0) and (self.CP.carFingerprint in [CAR.AP1_MODELS,CAR.AP2_MODELS]):
            #send this values so we can enable at 0 km/h
            tesla_accel_limits = [-1.4000000000000004,1.8000000000000007]
            tesla_jerk_limits = [-0.46000000000000085,0.47600000000000003]
            if self.CP.carFingerprint == CAR.AP2_MODELS:
                messages.append(self.tesla_can.create_ap2_long_control(350.0/3.6, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))
            if self.CP.carFingerprint == CAR.AP1_MODELS:
                messages.append(self.tesla_can.create_ap1_long_control(350.0/3.6, tesla_accel_limits, tesla_jerk_limits, CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter))

        return messages
