#!/usr/bin/env python3
from enum import Enum


class LongTunes(Enum):
  PEDAL = 0
  ACC = 1
  AP = 2

PEDAL_MIN = -7.
PEDAL_MAX = 65.

# MPH       0   11  44  67    90
PEDAL_BP = [  0.,  5., 20., 30., 40.]  # m/s
PEDAL_V = [ [35., 40., 60., 70., 85.],
            [25., 30., 45., 50., 55.],
            [20., 25., 35., 40., 50.],
            [20., 20., 27., 30., 40.],
]

###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.PEDAL:
    tune.kpBP = [0.0, 5.0, 22.0, 35.0]
    tune.kiBP = [0.0, 5.0, 22.0, 35.0]
    tune.kpV = [0.50, 0.45, 0.4, 0.4]
    tune.kiV = [0.01, 0.01, 0.01, 0.01]
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
