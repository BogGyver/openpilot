#!/usr/bin/env python3
from enum import Enum


class LongTunes(Enum):
  PEDAL = 0
  ACC = 1
  AP = 2


###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.PEDAL:
    tune.kpBP = [0., 5., 20.]
    tune.kpV = [0.1, 0.07, 0.05]
    tune.kiBP = [0., 5., 12., 20., 27.]
    tune.kiV = [.001, .0005, 0.0001, .00008, .00005]
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
