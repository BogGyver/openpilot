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
    factor = 6.
    if not perf:
      tune.kpBP = [0.0, 5.0, 22.0, 35.0]
      tune.kiBP = [0.0, 5.0, 22.0, 35.0]
      tune.kpV = [0.50/factor, 0.45/factor, 0.40/factor, 0.40/factor]
      tune.kiV = [0.01/factor, 0.01/factor, 0.01/factor, 0.01/factor]
    else:
      tune.kpBP = [0.0, 5.0, 22.0, 35.0]
      tune.kiBP = [0.0, 5.0, 22.0, 35.0]
      tune.kpV = [0.30/factor, 0.30/factor, 0.35/factor, 0.37/factor]
      tune.kiV = [0.007/factor, 0.007/factor, 0.0093/factor, 0.0092/factor]
      #some old ones to test
      #tune.kpV = [0.375, 0.325, 0.325, 0.325]
      #tune.kiV = [0.00915, 0.00825, 0.00725, 0.00725]
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
