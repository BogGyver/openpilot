#!/usr/bin/env python3
from enum import Enum
from selfdrive.car.modules.CFG_module import load_bool_param



class LongTunes(Enum):
  PEDAL = 0
  ACC = 1
  AP = 2

PEDAL_MIN = -7.
PEDAL_MAX = 65.

factor1 = 0.5
factor2 = 0.01
factor3 = 0.02
#for real PID used for pedal
# MPH         0   11    50    80
# km/h        0   18    80   126
pedal_kpBP = [0.0, 5.0, 22.0, 35.0]
pedal_kiBP = [0.0, 5.0, 22.0, 35.0]
pedal_kdBP = [0.0, 5.0, 22.0, 35.0]
pedal_kpV = [1.*factor1, 1.*factor1, 0.8*factor1, 0.8*factor1]
pedal_kiV = [1.*factor2, 1.*factor2, 0.9*factor2, 0.8*factor2]
pedal_kdV = [1.*factor3, 1.*factor3, 1.1*factor3, 1.25*factor3]
V_PID_FILE = "/data/params/pidParams"

# MPH       0   11  44  67    90
PEDAL_BP = [0., 5., 20., 30., 40.]  # m/s
PEDAL_V = [[60. , 60., 60., 60., 60.], #S60, maybe S70
           [60. , 60., 60., 60., 60.], #S85, SD85/90, Maybe SP85
           [60. , 60., 60., 60., 60.], #SP+ SPD89, etc
           ]
#PEDAL_V = [[37. , 43., 50., 55., 60.], #S60, maybe S70
#           [37. , 43., 50., 55., 60.], #S85, SD85/90, Maybe SP85
#           [37. , 43., 50., 55., 60.], #SP+ SPD89, etc
#           ]

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
      tune.kiV = [0.007, 0.007, 0.0053, 0.0052]
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
