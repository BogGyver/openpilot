from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

MAX_ANGLE = 87.  # make sure we never command the extremes (0xfff) which cause latching fault

class CAR:
  FUSION = "FORD FUSION 2018"

FINGERPRINTS = {
  CAR.FUSION: [{
    71: 8, 74: 8, 75: 8, 76: 8, 90: 8, 92: 8, 93: 8, 118: 8, 119: 8, 120: 8, 125: 8, 129: 8, 130: 8, 131: 8, 132: 8, 133: 8, 145: 8, 146: 8, 357: 8, 359: 8, 360: 8, 361: 8, 376: 8, 390: 8, 391: 8, 392: 8, 394: 8, 512: 8, 514: 8, 516: 8, 531: 8, 532: 8, 534: 8, 535: 8, 560: 8, 578: 8, 604: 8, 613: 8, 673: 8, 827: 8, 848: 8, 934: 8, 935: 8, 936: 8, 947: 8, 963: 8, 970: 8, 972: 8, 973: 8, 984: 8, 992: 8, 994: 8, 997: 8, 998: 8, 1003: 8, 1034: 8, 1045: 8, 1046: 8, 1053: 8, 1054: 8, 1058: 8, 1059: 8, 1068: 8, 1072: 8, 1073: 8, 1082: 8, 1107: 8, 1108: 8, 1109: 8, 1110: 8, 1200: 8, 1427: 8, 1430: 8, 1438: 8, 1459: 8
  }],
}

ECU_FINGERPRINT = {
  Ecu.fwdCamera: [970, 973, 984]
}

DBC = {
  CAR.FUSION: dbc_dict('ford_fusion_2018_pt', 'ford_fusion_2018_adas'),
}
