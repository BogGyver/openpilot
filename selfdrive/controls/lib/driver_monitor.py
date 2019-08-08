import numpy as np
from common.realtime import sec_since_boot, DT_CTRL, DT_DMON
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET
from common.filter_simple import FirstOrderFilter

_AWARENESS_TIME = 180        # 3 minutes limit without user touching steering wheels make the car enter a terminal status
_AWARENESS_PRE_TIME = 20.    # a first alert is issued 20s before expiration
_AWARENESS_PROMPT_TIME = 5.  # a second alert is issued 5s before start decelerating the car
_DISTRACTED_TIME = 7.
_DISTRACTED_PRE_TIME = 4.
_DISTRACTED_PROMPT_TIME = 2.
# model output refers to center of cropped image, so need to apply the x displacement offset
_FACE_THRESHOLD = 0.4
_PITCH_WEIGHT = 1.5  # pitch matters a lot more
_METRIC_THRESHOLD = 0.4
_PITCH_POS_ALLOWANCE = 0.08   # rad, to not be too sensitive on positive pitch
_PITCH_NATURAL_OFFSET = 0.1   # people don't seem to look straight when they drive relaxed, rather a bit up
_YAW_NATURAL_OFFSET = 0.08    # people don't seem to look straight when they drive relaxed, rather a bit to the right (center of car)
_STD_THRESHOLD = 0.1          # above this standard deviation consider the measurement invalid
_DISTRACTED_FILTER_TS = 0.25  # 0.6Hz
_VARIANCE_FILTER_TS = 20.     # 0.008Hz

MAX_TERMINAL_ALERTS = 3      # not allowed to engage after 3 terminal alerts
RESIZED_FOCAL = 320.0
H, W, FULL_W = 320, 160, 426

def head_orientation_from_descriptor(angles_desc, pos_desc):
  # the output of these angles are in device frame
  # so from driver's perspective, pitch is up and yaw is right
  # TODO this should be calibrated
  pitch_prnet = angles_desc[0]
  yaw_prnet = angles_desc[1]
  roll_prnet = angles_desc[2]

  face_pixel_position = ((pos_desc[0] + .5)*W - W + FULL_W, (pos_desc[1]+.5)*H)
  yaw_focal_angle = np.arctan2(face_pixel_position[0] - FULL_W/2, RESIZED_FOCAL)
  pitch_focal_angle = np.arctan2(face_pixel_position[1] - H/2, RESIZED_FOCAL)

  roll = roll_prnet
  pitch = pitch_prnet + pitch_focal_angle
  yaw = -yaw_prnet + yaw_focal_angle
  return np.array([roll, pitch, yaw])


class _DriverPose():
  def __init__(self):
    self.yaw = 0.
    self.pitch = 0.
    self.roll = 0.
    self.yaw_offset = 0.
    self.pitch_offset = 0.


def _monitor_hysteresis(variance_level, monitor_valid_prev):
  var_thr = 0.63 if monitor_valid_prev else 0.37
  return variance_level < var_thr


class DriverStatus():
  def __init__(self, monitor_on=False):
    self.pose = _DriverPose()
    self.monitor_on = monitor_on
    self.monitor_param_on = monitor_on
    self.monitor_valid = True   # variance needs to be low
    self.awareness = 1.
    self.driver_distracted = False
    self.driver_distraction_filter = FirstOrderFilter(0., _DISTRACTED_FILTER_TS, DT_DMON)
    self.variance_high = False
    self.variance_filter = FirstOrderFilter(0., _VARIANCE_FILTER_TS, DT_DMON)
    self.ts_last_check = 0.
    self.face_detected = False
    self.terminal_alert_cnt = 0
    self._set_timers()

  def _reset_filters(self):
    self.driver_distraction_filter.x = 0.
    self.variance_filter.x = 0.
    self.monitor_valid = True

  def _set_timers(self):
    if self.monitor_on:
      self.threshold_pre = _DISTRACTED_PRE_TIME / _DISTRACTED_TIME
      self.threshold_prompt = _DISTRACTED_PROMPT_TIME / _DISTRACTED_TIME
      self.step_change = DT_CTRL / _DISTRACTED_TIME
    else:
      self.threshold_pre = _AWARENESS_PRE_TIME / _AWARENESS_TIME
      self.threshold_prompt = _AWARENESS_PROMPT_TIME / _AWARENESS_TIME
      self.step_change = DT_CTRL / _AWARENESS_TIME

  def _is_driver_distracted(self, pose):
    # to be tuned and to learn the driver's normal pose
    pitch_error = pose.pitch - _PITCH_NATURAL_OFFSET
    yaw_error = pose.yaw - _YAW_NATURAL_OFFSET
    # add positive pitch allowance
    if pitch_error > 0.:
      pitch_error = max(pitch_error - _PITCH_POS_ALLOWANCE, 0.)
    pitch_error *= _PITCH_WEIGHT
    metric = np.sqrt(yaw_error**2 + pitch_error**2)
    # TODO: do something with the eye states and combine them with head pose
    return 1 if metric > _METRIC_THRESHOLD else 0


  def get_pose(self, driver_monitoring, params):
    if len(driver_monitoring.faceOrientation) == 0 or len(driver_monitoring.facePosition) == 0:
      return

    self.pose.roll, self.pose.pitch, self.pose.yaw = head_orientation_from_descriptor(driver_monitoring.faceOrientation, driver_monitoring.facePosition)
    self.face_detected = driver_monitoring.faceProb > _FACE_THRESHOLD

    self.driver_distracted = self._is_driver_distracted(self.pose)
    # first order filters
    self.driver_distraction_filter.update(self.driver_distracted)
    self.variance_high = False #driver_monitoring.std > _STD_THRESHOLD

    self.variance_filter.update(self.variance_high)

    monitor_param_on_prev = self.monitor_param_on
    monitor_valid_prev = self.monitor_valid

    # don't check for param too often as it's a kernel call
    ts = sec_since_boot()
    if ts - self.ts_last_check > 1.:
      self.monitor_param_on = params.get("IsDriverMonitoringEnabled") == "1"
      self.ts_last_check = ts

    self.monitor_valid = _monitor_hysteresis(self.variance_filter.x, monitor_valid_prev)
    self.monitor_on = self.monitor_valid and self.monitor_param_on
    if monitor_param_on_prev != self.monitor_param_on:
      self._reset_filters()
    self._set_timers()


  def update(self, events, driver_engaged, ctrl_active, standstill):

    driver_engaged |= (self.driver_distraction_filter.x < 0.37 and self.monitor_on)
    awareness_prev = self.awareness

    if (driver_engaged and self.awareness > 0.) or not ctrl_active:
      # always reset if driver is in control (unless we are in red alert state) or op isn't active
      self.awareness = 1.

    # only update if face is detected, driver is distracted and distraction filter is high
    if (not self.monitor_on or (self.driver_distraction_filter.x > 0.63 and self.driver_distracted and self.face_detected)) and \
       not (standstill and self.awareness - self.step_change <= self.threshold_prompt):
      self.awareness = max(self.awareness - self.step_change, -0.1)

    alert = None
    if self.awareness < 0.:
      # terminal red alert: disengagement required
      alert = 'driverDistracted' if self.monitor_on else 'driverUnresponsive'
      if awareness_prev >= 0.:
        self.terminal_alert_cnt += 1
    elif self.awareness <= self.threshold_prompt:
      # prompt orange alert
      alert = 'promptDriverDistracted' if self.monitor_on else 'promptDriverUnresponsive'
    elif self.awareness <= self.threshold_pre:
      # pre green alert
      alert = 'preDriverDistracted' if self.monitor_on else 'preDriverUnresponsive'

    if alert is not None:
      events.append(create_event(alert, [ET.WARNING]))

    return events


if __name__ == "__main__":
  ds = DriverStatus(True)
  ds.driver_distraction_filter.x = 0.
  ds.driver_distracted = 1
  for i in range(10):
    ds.update([], False, True, False)
    print(ds.awareness, ds.driver_distracted, ds.driver_distraction_filter.x)
  ds.update([], True, True, False)
  print(ds.awareness, ds.driver_distracted, ds.driver_distraction_filter.x)
