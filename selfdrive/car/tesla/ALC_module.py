from common.numpy_fast import clip, interp
from selfdrive.car.tesla.values import CarControllerParams, CAR, CAN_CHASSIS, CAN_AUTOPILOT, CAN_EPAS, CAN_POWERTRAIN
from cereal import log
import numpy as np

IC_LANE_SCALE = 0.5
LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

class ALCController:

    def __init__ (self):
        self.alca_engaged_frame = 0
        

    def update(self, enabled, CS, frame, lat_plan):
        #get lat plan info
        
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