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
PEDAL_V = [ [85., 85., 85., 85., 85., 85.],
            [32., 37., 45., 55., 65., 70.],
            [25., 30., 35., 40., 45., 50.],
            [20., 25., 30., 35., 40., 45.],
            [66., 66., 66., 66., 66., 66.],
]
#MPH                    0     16    33    55    90
ACCEL_LOOKUP_BP =     [ 0.0,  7.5, 15.0, 25.0, 40.0]
ACCEL_MAX_LOOKUP_V  = [[ 2.0,  1.0,  0.6,  0.4,  0.3],
                       [ 2.2,  1.4,  1.0,  0.7,  0.5],
                       [ 2.5,  1.9,  1.5,  1.2,  1.0],
]
ACCEL_MIN_LOOKUP_V =  [TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL]
ACCEL_REG_LOOKUP_V =  [ -1.5, -1.5, -1.5, -1.5, -1.5]


###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.PEDAL:
    tune.kpBP = [0.0, 5.0, 22.0, 35.0]
    tune.kiBP = [0.0, 5.0, 22.0, 35.0]
    tune.kpV = [0.50, 0.50, 0.50, 0.50]
    tune.kiV = [0.05, 0.05, 0.05, 0.05]
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
