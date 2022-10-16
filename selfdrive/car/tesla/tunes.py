#!/usr/bin/env python3
from enum import Enum
from selfdrive.car.modules.CFG_module import load_bool_param


class LongTunes(Enum):
  PEDAL = 0
  ACC = 1
  AP = 2

factor1 = 0.05
factor2 = 0.2
factor3 = factor2 * 0 #.25
#for real PID
kdBp = [0, 5., 22.,35.]
kdV = [1.5*factor3, 1.5*factor3, 1.5*factor3, 1.5*factor3]
V_PID_FILE = "/data/params/pidParams"
gasMaxBP = [0.0, 20.0]  # m/s
gasMaxV = [
            0.125,
            0.425,
        ]  # if ret.enableGasInterceptor else [0.] # max gas allowed
brakeMaxBP = [0.0]  # m/s
brakeMaxV = [
            -0.07
        ]  # max brake allowed - BB: since we are using regen, make this even

###### LONG ######
def set_long_tune(tune, name):
  # Improved longitudinal tune
  if name == LongTunes.PEDAL:
    perf = load_bool_param("TinklaHasPerfMotor",False)
    
    if not perf:
      # tune.kpBP = [0.0, 5.0, 22.0, 35.0]
      # tune.kiBP = [0.0, 5.0, 22.0, 35.0]
      # tune.kpV = [0.50/factor1, 0.45/factor1, 0.40/factor1, 0.40/factor1]
      # tune.kiV = [0.01/factor2, 0.01/factor2, 0.01/factor2, 0.01/factor2]
      # MPH         0   11    50   80
      # km/h        0   18    80   126
      tune.kpBP = [0.0, 5.0, 22.0, 35.0]
      tune.kiBP = [0.0, 5.0, 22.0, 35.0]
      tune.kpV = [1.*factor1, 1.*factor1, 0.7*factor1, 0.5*factor1]
      tune.kiV = [1.*factor2, 1.*factor2, 0.7*factor2, 0.5*factor2]
    else:
      tune.kpBP = [0.0, 5.0, 22.0, 35.0]
      tune.kiBP = [0.0, 5.0, 22.0, 35.0]
      tune.kpV = [0.30/factor1, 0.30/factor1, 0.35/factor1, 0.37/factor1]
      tune.kiV = [0.007/factor2, 0.007/factor2, 0.0093/factor2, 0.0092/factor2]
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
