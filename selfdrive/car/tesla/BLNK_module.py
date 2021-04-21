class Blinker:
    def __init__(self):
        # stalk signal for less than 550ms means it was tapped
        self.tap_duration_frames = 55  
        self.tap_direction = 0 
        self.blinker_on_frame_start = 0
        self.blinker_on_frame_end = 0
        self.prev_turn_signal_stalk_state = 0

    def update_state(self, CS, frame):
        #opposite direction so start canceling
        if (
            self.tap_direction > 0  
            and CS.turn_signal_stalk_state > 0 
            and self.tap_direction != CS.turn_signal_stalk_state
        ):
            self.tap_direction = 0
            self.blinker_on_frame_start = 0
            self.blinker_on_frame_end = 0

        # turn signal stalk just turned on, capture frame
        if (
            CS.turn_signal_stalk_state > 0 
            and self.prev_turn_signal_stalk_state == 0
        ):
            self.blinker_on_frame_start = frame
        # turn signal stalk just turned off
        elif (
            CS.turn_signal_stalk_state == 0
            and self.prev_turn_signal_stalk_state > 0
        ):  
            if frame - self.blinker_on_frame_start <= self.tap_duration_frames:
                #recognize tap
                self.tap_direction = self.prev_turn_signal_stalk_state
                self.blinker_on_frame_end = frame 
            else:
                #too long, no tap
                self.tap_direction = 0
                self.blinker_on_frame_start = 0
                self.blinker_on_frame_end = 0

        # check what we need to maintain
        if (
            self.tap_direction > 0
            and frame - self.blinker_on_frame_start > self.tap_duration_frames
            and frame - self.blinker_on_frame_end > self.tap_duration_frames
            and CS.alca_direction != self.tap_direction
        ):
            self.tap_direction = 0
            self.blinker_on_frame_start = 0
            self.blinker_on_frame_end = 0

        self.prev_turn_signal_stalk_state = CS.turn_signal_stalk_state