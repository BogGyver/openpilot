#!/usr/bin/env python3
from enum import Enum
from selfdrive.car.tesla.values import TESLA_MIN_ACCEL

class LongTunes(Enum):
  PEDAL = 0
  ACC = 1
  AP = 2

PEDAL_MIN = -7.
PEDAL_MAX = 65.

# MPH         0   11   27   44   67   90
# km/h        0   18   43   72  108  144
PEDAL_BP = [  0.,  5., 12., 20., 30., 40.]  # m/s
PEDAL_V = [ [40., 45., 55., 65., 75., 85.],
            [30., 35., 42., 50., 55., 50.],
            [25., 30., 35., 40., 45., 50.],
            [20., 25., 30., 35., 40., 45.],
            [85., 85., 85., 85., 85., 85.],
]
#MPH                    0     16    33    55    90
ACCEL_LOOKUP_BP =     [ 0.0,  7.5, 15.0, 25.0, 40.0]

AP_ACCEL_MAX_V =     [ 2.0,  1.0,  0.8,  0.5,  0.3]
AP_ACCEL_MIN_V =     [TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL]

PREAP_ACCEL_MAX_V =   [ 2.0,  1.0,  0.8,  0.5,  0.3]
PREAP_ACCEL_MIN_V =   [ -1.5, -1.5, -1.5, -1.5, -1.5] #(regen only... for iBooster the values go to MAX

PREAP_IBST_ACCEL_MAX_V =   [ 2.0,  1.0,  0.8,  0.5,  0.3]
PREAP_IBST_ACCEL_MIN_V =   [TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL]

###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.PEDAL:
    tune.kpBP = [0.0, 5.0, 22.0, 35.0]
    tune.kiBP = [0.0, 5.0, 22.0, 35.0]
    tune.kpV = [0.50, 0.45, 0.4, 0.4]
    tune.kiV = [0.1, 0.1, 0.1, 0.1]
  # Default longitudinal tune
  elif name == LongTunes.ACC:
    tune.kpBP = [0]
    tune.kiBP = [0]
    tune.kpV = [0]
    tune.kiV = [0]
  elif name == LongTunes.AP:
    tune.kpBP = [0]
    tune.kiBP = [0]
    tune.kpV = [0]
    tune.kiV = [0]
  else:
    raise NotImplementedError('This longitudinal tune does not exist')
