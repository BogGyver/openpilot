#!/usr/bin/env python3
from enum import Enum
from selfdrive.car.tesla.values import TESLA_MIN_ACCEL
from selfdrive.car.modules.CFG_module import load_bool_param,load_float_param


class LongTunes(Enum):
  PEDAL = 0
  ACC = 1
  AP = 2
  IBST = 3

#######################################################
# Pedal calibration and translation - DO NOT CHANGE
#######################################################
PEDAL_MIN = load_float_param("TeslaPedalCalibMin",-3.)
PEDAL_MAX = load_float_param("TeslaPedalCalibMax",99.6)
PEDAL_FACTOR = load_float_param("TeslaPedalCalibFactor",1.)
PEDAL_ZERO = load_float_param("TeslaPedalCalibZero",0.) - 1 / PEDAL_FACTOR
PEDAL_CALIBRATED = load_bool_param("TeslaPedalCalibDone",False)
PEDAL_DI_MIN = -5
PEDAL_DI_ZERO = 0
PEDAL_DI_PRESSED = 2

def transform_pedal_to_di(val):
  return PEDAL_DI_ZERO + (val - PEDAL_ZERO) * PEDAL_FACTOR

def transform_di_to_pedal(val):
  return PEDAL_ZERO + (val - PEDAL_DI_ZERO) / PEDAL_FACTOR
#######################################################

#These values are in DI_PEDAL range
# MPH         0   11   27   44   67   90
# km/h        0   18   43   72  108  144
PEDAL_BP = [  0.,  5., 12., 20., 30., 40.]  # m/s
PEDAL_V = [ [99., 99., 99., 99., 99., 99.], #1-S60
            [55., 63., 75., 90., 99., 99.], #2-S85
            [45., 52., 60., 67., 75., 82.], #3-P85
            [37., 45., 52., 60., 67., 75.], #4-P85+
            [99., 99., 99., 99., 99., 99.], #5-Generic
]
#MPH                    0    3    16    33    55    90
ACCEL_LOOKUP_BP =     [ 0.0, 1.3, 7.5, 15.0, 25.0, 40.0]
ACCEL_MAX_LOOKUP_V  = [[0.3, 0.7, 0.9,  0.7,  0.6,  0.5], #1-Chill
                       [0.3, 0.9, 1.2,  1.0,  0.8,  0.6], #2-Standard
                       [0.3, 1.6, 1.9,  1.5,  1.2,  1.0], #3-MadMax
]
ACCEL_AP_MAX_LOOKUP_V  = [[2.0, 1.8, 1.2,  0.7,  0.6,  0.5], #1-Chill
                          [2.2, 2.0, 1.4,  1.0,  0.8,  0.6], #2-Standard
                          [2.5, 2.3, 1.9,  1.5,  1.2,  1.0], #3-MadMax
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
    tune.kpV = [0.50, 0.50, 0.45, 0.40]
    tune.kiV = [0.05, 0.05, 0.05, 0.05]
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
