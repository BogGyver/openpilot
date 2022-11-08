#!/usr/bin/env python3
from enum import Enum
from selfdrive.car.tesla.values import TESLA_MIN_ACCEL

class LongTunes(Enum):
  PEDAL = 0
  ACC = 1
  AP = 2
  IBST = 3

#These values are in DI_PEDAL range
# MPH         0   11   27   44   67   90
# km/h        0   18   43   72  108  144
PEDAL_BP = [  0.,  5., 12., 20., 30., 40.]  # m/s
PEDAL_V = [ [99., 99., 99., 99., 99., 99.],
            [55., 63., 75., 90., 99., 99.],
            [45., 52., 60., 67., 75., 82.],
            [37., 45., 52., 60., 67., 75.],
            [99., 99., 99., 99., 99., 99.],
]
#MPH                    0    3    16    33    55    90
ACCEL_LOOKUP_BP =     [ 0.0, 1.3, 7.5, 15.0, 25.0, 40.0]
ACCEL_MAX_LOOKUP_V  = [[0.3, 0.5, 0.9,  0.7,  0.6,  0.5],
                       [0.3, 0.6, 1.2,  1.0,  0.8,  0.6],
                       [0.3, 1.6, 1.9,  1.5,  1.2,  1.0],
]
ACCEL_MIN_LOOKUP_V =  [TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL, TESLA_MIN_ACCEL]
ACCEL_REG_LOOKUP_V =  [ -1.5, -1.5, -1.5, -1.5, -1.5, -1.5]


###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.PEDAL:
    tune.kpBP = [0.0, 5.0, 22.0, 35.0]
    tune.kiBP = [0.0, 5.0, 22.0, 35.0]
    tune.kpV = [0.75, 0.75, 0.75, 0.75]
    tune.kiV = [0.07, 0.07, 0.07, 0.07]
  # Default longitudinal tune
  elif name == LongTunes.IBST:
    tune.kpBP = [0.0, 5.0, 22.0, 35.0]
    tune.kiBP = [0.0, 5.0, 22.0, 35.0]
    tune.kpV = [0.60, 0.60, 0.60, 0.60]
    tune.kiV = [0.07, 0.07, 0.07, 0.07]
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
