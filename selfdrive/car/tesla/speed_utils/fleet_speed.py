from selfdrive.car.tesla.speed_utils.movingaverage import MovingAverage


class FleetSpeed:
    def __init__(self, average_speed_over_x_suggestions):
        self.speed_avg = MovingAverage(average_speed_over_x_suggestions)
        self.frame_last_adjustment = 0

    def adjust(self, CS, max_speed_ms, frame):
        if CS.mapAwareSpeed and self.is_valid(CS, max_speed_ms):
            self.frame_last_adjustment = frame
            # if max speed is greater than the speed limit, apply a relative offset to map speed
            if (
                CS.rampType == 0
                and max_speed_ms > CS.speed_limit_ms > CS.map_suggested_speed
            ):
                return self.speed_avg.add(
                    max_speed_ms * CS.map_suggested_speed / CS.speed_limit_ms
                )
            else:
                return self.speed_avg.add(CS.map_suggested_speed)
        return max_speed_ms

    def is_active(self, frame):
        return (
            self.frame_last_adjustment > 0 and frame <= self.frame_last_adjustment + 10
        )

    def reset_averager(self):
        self.speed_avg.reset()

    @classmethod
    def is_available(cls, CS):
        return (
            CS.mapAwareSpeed
            and CS.medianFleetSpeedMPS > 0
            and CS.splineLocConfidence > 60
            and CS.UI_splineID > 0
        )

    @classmethod
    def is_valid(cls, CS, max_speed_ms):
        if CS.map_suggested_speed <= 0 or CS.map_suggested_speed > max_speed_ms:
            return False
        if CS.speed_limit_ms == 0:  # no or unknown speed limit
            if (
                CS.rampType == 0 and CS.map_suggested_speed >= 17
            ):  # more than 61 kph / 38 mph, means we may be on a road without speed limit
                return False
        elif (
            CS.speed_limit_ms < CS.map_suggested_speed
        ):  # if map speed exceeds the speed limit, we'll ignore it
            return False
        return True