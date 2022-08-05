#!/usr/bin/env python3
from enum import Enum
from selfdrive.car.modules.CFG_module import load_bool_param


class LongTunes(Enum):
  PEDAL = 0
  ACC = 1
  AP = 2


###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.PEDAL:
    perf = load_bool_param("TinklaHasPerfMotor",False)
    if not perf:
      tune.kpBP = [0.0, 5.0, 22.0, 35.0]
      tune.kiBP = [0.0, 5.0, 22.0, 35.0]
      tune.kpV = [0.50, 0.45, 0.4, 0.4]
      tune.kiV = [0.01, 0.01, 0.01, 0.01]
    else:
      tune.kpBP = [0.0, 5.0, 22.0, 35.0]
      tune.kiBP = [0.0, 5.0, 22.0, 35.0]
      tune.kpV = [0.3, 0.3, 0.35, 0.37]
      tune.kiV = [0.007, 0.007, 0.0093, 0.0092]
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
