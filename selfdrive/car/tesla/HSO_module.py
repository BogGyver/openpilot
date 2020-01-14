#human steer override module

class HSOController():
  def __init__(self, carcontroller):
    self.human_control = False
    self.frame_humanSteered = 0

  def update_stat(self, CC, CS, enabled, actuators, frame):
    human_control = False

    if CS.enableHSO and enabled:
      # if steering but not by ALCA
      if CS.steer_override > 0:
        self.frame_humanSteered = frame
      elif CC.ALCA.laneChange_enabled <= 1 and frame > CC.blinker.blinker_on_frame_start + CC.blinker.tap_duration_frames:
        if CS.turn_signal_stalk_state > 0 or frame <= (CC.blinker.blinker_on_frame_start + int(100 * CS.hsoNumbPeriod)): # stalk locked or blinker within numbPeriod
         self.frame_humanSteered = frame
        elif frame - self.frame_humanSteered < 50:  # Need more human testing of handoff timing
          # Find steering difference between visiond model and human (no need to do every frame if we run out of CPU):
          apply_steer = int(-actuators.steerAngle)
          angle_diff = abs(apply_steer - CS.angle_steers)
          if angle_diff > 15.:
            self.frame_humanSteered = frame
      if frame - self.frame_humanSteered < 50:
        human_control = True
        CS.UE.custom_alert_message(3, "Manual Steering Enabled", 51, 4)

    if (not human_control) and (CC.DAS_219_lcTempUnavailableSpeed == 1):
      CC.DAS_219_lcTempUnavailableSpeed = 0
      CC.warningNeeded = 1
    self.human_control = human_control
    return human_control and enabled
