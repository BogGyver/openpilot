# human steer override module

class HSOController:
    def __init__(self):
        self.human_control = False
        self.frame_humanSteered = 0

    def update_stat(self, CS, enabled, actuators, frame):
        human_control = False

        if CS.enableHSO and enabled:
            # if steering but not by ALCA
            if CS.out.steeringPressed:
                self.frame_humanSteered = frame
            elif (frame - self.frame_humanSteered < 50) and (CS.out.leftBlinker or CS.out.rightBlinker):  
                # stalk locked, update frame
                self.frame_humanSteered = frame
            elif (frame - self.frame_humanSteered < 50):  
                # Need more human testing of handoff timing
                # Find steering difference between visiond model and human (no need to do every frame if we run out of CPU):
                apply_steer = int(actuators.steeringAngleDeg)
                angle_diff = abs(apply_steer - CS.out.steeringAngleDeg)
                if angle_diff > 15.0:
                    self.frame_humanSteered = frame
            if frame - self.frame_humanSteered < 50:
                human_control = True

        self.human_control = human_control
        return human_control and enabled
