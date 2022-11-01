#!/usr/bin/env python3
from enum import Enum


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
]

###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.PEDAL:
    tune.kpBP = [0.0, 5.0, 22.0, 35.0]
    tune.kiBP = [0.0, 5.0, 22.0, 35.0]
    tune.kpV = [0.50, 0.45, 0.4, 0.4]
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
