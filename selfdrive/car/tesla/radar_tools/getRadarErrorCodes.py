#!/usr/bin/env python3

import cereal.messaging as messaging
from selfdrive.car.interfaces import CarInterfaceBase
from opendbc.can.parser import CANParser
from selfdrive.car.tesla.values import DBC, CAR, CAN_RADAR


error_signals = ["RADC_a001_ecuInternalPerf","RADC_a002_flashPerformance","RADC_a003_vBatHigh",
    "RADC_a004_adjustmentNotDone","RADC_a005_adjustmentReq","RADC_a006_adjustmentNotOk",
    "RADC_a007_sensorBlinded","RADC_a008_plantModeActive","RADC_a009_configMismatch",
    "RADC_a010_canBusOff","RADC_a011_bdyMIA","RADC_a012_espMIA","RADC_a013_gtwMIA",
    "RADC_a014_sccmMIA","RADC_a015_adasMIA","RADC_a016_bdyInvalidCount",
    "RADC_a017_adasInvalidCount","RADC_a018_espInvalidCount","RADC_a019_sccmInvalidCount",
    "RADC_a020_bdyInvalidChkSm","RADC_a021_espInvalidChkSm","RADC_a022_sccmInvalidChkSm",
    "RADC_a023_sccmInvalidChkSm","RADC_a024_absValidity","RADC_a025_ambTValidity",
    "RADC_a026_brakeValidity","RADC_a027_CntryCdValidity","RADC_a028_espValidity",
    "RADC_a029_longAccOffValidity","RADC_a030_longAccValidity","RADC_a031_odoValidity",
    "RADC_a032_gearValidity","RADC_a033_steerAngValidity","RADC_a034_steerAngSpdValidity",
    "RADC_a035_indctrValidity","RADC_a036_vehStandStillValidity","RADC_a037_vinValidity",
    "RADC_a038_whlRotValidity","RADC_a039_whlSpdValidity","RADC_a040_whlStandStillValidity",
    "RADC_a041_wiperValidity","RADC_a042_xwdValidity","RADC_a043_yawOffValidity",
    "RADC_a044_yawValidity","RADC_a045_bsdSanity","RADC_a046_rctaSanity",
    "RADC_a047_lcwSanity","RADC_a048_steerAngOffSanity","RADC_a049_tireSizeSanity",
    "RADC_a050_velocitySanity","RADC_a051_yawSanity","RADC_a052_radomeHtrInop",
    "RADC_a053_espmodValidity","RADC_a054_gtwmodValidity","RADC_a055_stwmodValidity",
    "RADC_a056_bcmodValidity","RADC_a057_dimodValidity","RADC_a058_opmodValidity",
    "RADC_a059_drmiInvalidChkSm","RADC_a060_drmiInvalidCount","RADC_a061_radPositionMismatch",
    "RADC_a062_strRackMismatch","unused62"]
error_message = "TeslaRadarAlertMatrix"

def get_radar_error_parser(CP):
  # Status messages
  signals = []
  checks = [('TeslaRadarAlertMatrix', 1),]
  for signal in error_signals:
    tpl = (signal, error_message)
    signals.append(tpl)
  return CANParser(DBC[CP.carFingerprint]['radar'], signals, checks, CAN_RADAR[CP.carFingerprint])
  

if __name__ == "__main__":
  CP = CarInterfaceBase.get_std_params(CAR.PREAP_MODELS)
  rcp = get_radar_error_parser(CP)
  can_sock = messaging.sub_sock('can')
  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    values = rcp.update_strings(can_strings)
    errors = 0
    print(chr(27) + "[2J")
    print("Checking TeslaRadarAlertMatrix for errors...")
    print("--------------------------------------------")
    for signal in error_signals:
      val = rcp.vl[error_message][signal]
      if val > 0:
        print(signal,val)
        errors = errors + 1
    if errors == 0:
      print ("No errors!")
    else:
      print(errors," errors!")


