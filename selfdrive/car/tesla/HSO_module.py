#human steer override module

class HSOController():
    def __init__(self,carcontroller):
        self.human_control = False
        self.frame_humanSteered = 0

    def update_stat(self,CC,CS,enabled,actuators,frame):
        human_control = False

        if CS.enableHSO and enabled:
          #if steering but not by ALCA
          if CS.blinker_on and (CC.ALCA.laneChange_enabled <= 1):
            self.frame_humanSteered = frame
          if (CS.steer_override > 0): # and (frame - self.frame_humanSteered > 50): #let's try with human touch only
            self.frame_humanSteered = frame
          else:
            if (CC.ALCA.laneChange_enabled <= 1) and (frame - self.frame_humanSteered < 50): # Need more human testing of handoff timing
              # Find steering difference between visiond model and human (no need to do every frame if we run out of CPU):
              steer_current=(CS.angle_steers)  # Formula to convert current steering angle to match apply_steer calculated number
              apply_steer = int(-actuators.steerAngle)
              angle = abs(apply_steer-steer_current)
              if frame < (CC.blinker_on_frame_start + int(100 * CS.hsoNumbPeriod)) or angle > 15.:
                self.frame_humanSteered = frame
          if (frame - self.frame_humanSteered < 50):
            human_control = True
            CS.UE.custom_alert_message(3,"Manual Steering Enabled",51,4)

        if (not human_control) and (CC.DAS_219_lcTempUnavailableSpeed == 1):
          CC.DAS_219_lcTempUnavailableSpeed = 0
          CC.warningNeeded = 1
        self.human_control = human_control
        return human_control and enabled
