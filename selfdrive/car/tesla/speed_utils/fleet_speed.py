from selfdrive.car.tesla.speed_utils.movingaverage import MovingAverage


class FleetSpeed:

  def __init__(self, average_speed_over_x_suggestions):
    self.speed_avg = MovingAverage(average_speed_over_x_suggestions)

  def adjust(self, CS, max_speed_ms):
    if CS.mapAwareSpeed and self.is_valid(CS, max_speed_ms):
      # if max speed is greater than the speed limit, apply a relative offset to map speed
      if CS.rampType == 0 and max_speed_ms > CS.baseMapSpeedLimitMPS > CS.map_suggested_speed:
        return self.speed_avg.add(max_speed_ms * CS.map_suggested_speed / CS.baseMapSpeedLimitMPS)
      else:
        return self.speed_avg.add(CS.map_suggested_speed)
    return max_speed_ms

  def reset_averager(self):
    self.speed_avg.reset()

  @classmethod
  def is_valid(cls, CS, max_speed_ms):
    if CS.map_suggested_speed <= 0 or CS.map_suggested_speed > max_speed_ms:
      return False
    if CS.baseMapSpeedLimitMPS == 0: # no or unknown speed limit
      if CS.rampType == 0 and CS.map_suggested_speed >= 17: # more than 61 kph / 38 mph, means we may be on a road without speed limit
        return False
    elif CS.baseMapSpeedLimitMPS < CS.map_suggested_speed: # if map speed exceeds the speed limit, we'll ignore it
      return False
    return True
