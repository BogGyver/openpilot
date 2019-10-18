#human steer override module
import time

MAX_TIME_BLINKER_ON = 150 # in 0.01 seconds
TIME_REMEMBER_LAST_BLINKER = 50 # in 0.01 seconds

def _current_time_millis():
  return int(round(time.time() * 1000))

class HSOController():
    def __init__(self,carcontroller):
        self.CC = carcontroller
        self.frame_humanSteered = 0
        self.turn_signal_needed = 0 # send 1 for left, 2 for right 0 for not needed
        self.last_blinker_on = 0
        self.blinker_on = 0
        self.frame_blinker_on = 0
        self.last_human_blinker_on = 0
        self.frame_human_blinker_on = 0
    

    def update_stat(self,CC,CS,enabled,actuators,frame):
        human_control = False
        self.last_blinker_on = self.blinker_on
        if CS.right_blinker_on:
          self.blinker_on = 2
          self.frame_human_blinker_on = frame
        elif CS.left_blinker_on:
          self.blinker_on = 1
          self.frame_human_blinker_on = frame
        if self.blinker_on > 0:
            self.frame_blinker_on = frame
            self.last_human_blinker_on = self.blinker_on
        else:
            if frame - self.frame_blinker_on < TIME_REMEMBER_LAST_BLINKER:
                self.blinker_on = self.last_human_blinker_on
            else:
                self.last_human_blinker_on = 0
        if frame - self.frame_human_blinker_on > MAX_TIME_BLINKER_ON:
          self.blinker_on = 0
          self.turn_signal_needed = 0
          self.last_blinker_on = 0

        if CS.enableHSO and enabled:
          #if steering but not by ALCA
          if (CS.right_blinker_on or CS.left_blinker_on) and (self.CC.ALCA.laneChange_enabled <= 1):# and (self.last_blinker_on != self.blinker_on):
            self.frame_humanSteered = frame
          if (CS.steer_override > 0) and (frame - self.frame_humanSteered > 50): 
            self.frame_humanSteered = frame
          else:
            if (frame - self.frame_humanSteered < 50): # Need more human testing of handoff timing
              # Find steering difference between visiond model and human (no need to do every frame if we run out of CPU):
              steer_current=(CS.angle_steers)  # Formula to convert current steering angle to match apply_steer calculated number
              apply_steer = int(-actuators.steerAngle)
              angle = abs(apply_steer-steer_current)
              if angle > 15.:
                self.frame_humanSteered = frame
        if enabled:
            if CS.enableHSO:
              if (frame - self.frame_humanSteered < 50):
                human_control = True
                CS.UE.custom_alert_message(3,"Manual Steering Enabled",51,4)
        #if human control and turn signal on, and previous turn signal was not on or was different, adjust
        #same direction again cancels signal
        if human_control:
          self.turn_signal_needed = self.blinker_on
        #if no human control, no turn signal needed
        if not human_control:
          self.turn_signal_needed = 0
          self.last_blinker_on = 0
          self.blinker_on = 0
        if (not human_control) and (CC.DAS_219_lcTempUnavailableSpeed == 1):
          CC.DAS_219_lcTempUnavailableSpeed = 0
          CC.warningNeeded = 1
          self.turn_signal_needed = 0
        if (self.CC.ALCA.laneChange_enabled > 1):
          self.turn_signal_needed = 0
          self.blinker_on = 0
          self.last_blinker_on = 0
        return human_control and enabled, self.turn_signal_needed
