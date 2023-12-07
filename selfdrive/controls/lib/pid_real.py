import numpy as np
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.car.tesla.speed_utils.movingaverage import MovingAverage


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

class PIDController():
  def __init__(self, k_p, k_i, k_d, k_f=0.85, pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, convert=None):
    self._k_p = k_p # proportional gain
    self._k_i = k_i # integral gain
    self._k_d = k_d # derivative gain
    self.k_f = k_f  # feedforward gain

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.rate = 1.0 / rate
    self.d_rate = 7.0 / rate
    self.sat_limit = sat_limit
    self.convert = convert
    self.past_errors_avg = 0
    self.past_errors = MovingAverage(3)

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])


  def _check_saturation(self, control, override, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and not override and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.f = 0.0
    self.d = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    self.past_errors_avg = 0
    self.past_errors.reset()

  def update(self, setpoint, measurement, speed=0.0, override=False, feedforward=0., deadzone=0., freeze_integrator=False,check_saturation=True, ):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * self.k_p

    #clip the feedforward during the last 5 kmh for smooth transition
    clipped_error = clip(error,0,5)
    self.k_f = clipped_error * 0.2

    self.f = feedforward * self.k_f
    self.d = 0.0
    if self.past_errors.full():
      self.d = self.k_d * ((error - self.past_errors_avg) / self.d_rate)
    self.past_errors_avg = self.past_errors.add(error)

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + self.f + i + self.d

      if self.convert is not None:
        control = self.convert(control, speed=self.speed)

      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or \
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i
      else:
        control = self.p + self.f + self.i + self.d
        if self.convert is not None:
          control = self.convert(control, speed=self.speed)

    if check_saturation:
      self.saturated = self._check_saturation(control, override, error)
    else:
      self.saturated = False

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control