from cereal import car
from selfdrive.car import dbc_dict

Ecu = car.CarParams.Ecu
VisualAlert = car.CarControl.HUDControl.VisualAlert

# Car button codes
class CruiseButtons:
  RES_ACCEL   = 4
  DECEL_SET   = 3
  CANCEL      = 2
  MAIN        = 1

# See dbc files for info on values"
VISUAL_HUD = {
  VisualAlert.none: 0,
  VisualAlert.fcw: 1,
  VisualAlert.steerRequired: 1,
  VisualAlert.brakePressed: 10,
  VisualAlert.wrongGear: 6,
  VisualAlert.seatbeltUnbuckled: 5,
  VisualAlert.speedTooHigh: 8}

class CAR:
  ACCORD = "HONDA ACCORD 2018 SPORT 2T"
  ACCORD_15 = "HONDA ACCORD 2018 LX 1.5T"
  ACCORDH = "HONDA ACCORD 2018 HYBRID TOURING"
  CIVIC = "HONDA CIVIC 2016 TOURING"
  CIVIC_BOSCH = "HONDA CIVIC HATCHBACK 2017 SEDAN/COUPE 2019"
  CIVIC_BOSCH_DIESEL = "HONDA CIVIC SEDAN 1.6 DIESEL"
  ACURA_ILX = "ACURA ILX 2016 ACURAWATCH PLUS"
  CRV = "HONDA CR-V 2016 TOURING"
  CRV_5G = "HONDA CR-V 2017 EX"
  CRV_EU = "HONDA CR-V 2016 EXECUTIVE"
  CRV_HYBRID = "HONDA CR-V 2019 HYBRID"
  FIT = "HONDA FIT 2018 EX"
  ODYSSEY = "HONDA ODYSSEY 2018 EX-L"
  ODYSSEY_CHN = "HONDA ODYSSEY 2019 EXCLUSIVE CHN"
  ACURA_RDX = "ACURA RDX 2018 ACURAWATCH PLUS"
  PILOT = "HONDA PILOT 2017 TOURING"
  PILOT_2019 = "HONDA PILOT 2019 ELITE"
  RIDGELINE = "HONDA RIDGELINE 2017 BLACK EDITION"
  INSIGHT = "HONDA INSIGHT 2019 TOURING"

# diag message that in some Nidec cars only appear with 1s freq if VIN query is performed
DIAG_MSGS = {1600: 5, 1601: 8}

FINGERPRINTS = {
  CAR.ACCORD: [{
    148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 399: 7, 419: 8, 420: 8, 427: 3, 432: 7, 441: 5, 446: 3, 450: 8, 464: 8, 477: 8, 479: 8, 495: 8, 545: 6, 662: 4, 773: 7, 777: 8, 780: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 927: 8, 929: 8, 1302: 8, 1600: 5, 1601: 8, 1652: 8
  }],
  CAR.ACCORD_15: [{
    148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 427: 3, 432: 7, 441: 5, 446: 3, 450: 8, 464: 8, 477: 8, 479: 8, 495: 8, 545: 6, 662: 4, 773: 7, 777: 8, 780: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 927: 8, 929: 8, 1302: 8, 1600: 5, 1601: 8, 1652: 8
  }],
  CAR.ACCORDH: [{
    148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 387: 8, 388: 8, 399: 7, 419: 8, 420: 8, 427: 3, 432: 7, 441: 5, 450: 8, 464: 8, 477: 8, 479: 8, 495: 8, 525: 8, 545: 6, 662: 4, 773: 7, 777: 8, 780: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 927: 8, 929: 8, 1302: 8, 1600: 5, 1601: 8, 1652: 8
  }],
  CAR.ACURA_ILX: [{
    57: 3, 145: 8, 228: 5, 304: 8, 316: 8, 342: 6, 344: 8, 380: 8, 398: 3, 399: 7, 419: 8, 420: 8, 422: 8, 428: 8, 432: 7, 464: 8, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 542: 7, 545: 4, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 829: 5, 882: 2, 884: 7, 887: 8, 888: 8, 892: 8, 923: 2, 929: 4, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1030: 5, 1034: 5, 1036: 8, 1039: 8, 1057: 5, 1064: 7, 1108: 8, 1365: 5,
  }],
  # Acura RDX w/ Added Comma Pedal Support (512L & 513L)
  CAR.ACURA_RDX: [{
    57: 3, 145: 8, 229: 4, 308: 5, 316: 8, 342: 6, 344: 8, 380: 8, 392: 6, 398: 3, 399: 6, 404: 4, 420: 8, 422: 8, 426: 8, 432: 7, 464: 8, 474: 5, 476: 4, 487: 4, 490: 8, 506: 8, 512: 6, 513: 6, 542: 7, 545: 4, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 829: 5, 882: 2, 884: 7, 887: 8, 888: 8, 892: 8, 923: 2, 929: 4, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1033: 5, 1034: 5, 1036: 8, 1039: 8, 1057: 5, 1064: 7, 1108: 8, 1365: 5, 1424: 5, 1729: 1
  }],
  CAR.CIVIC: [{
    57: 3, 148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 427: 3, 428: 8, 432: 7, 450: 8, 464: 8, 470: 2, 476: 7, 487: 4, 490: 8, 493: 5, 506: 8, 512: 6, 513: 6, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 892: 8, 927: 8, 929: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1108: 8, 1302: 8, 1322: 5, 1361: 5, 1365: 5, 1424: 5, 1633: 8,
  }],
  CAR.CIVIC_BOSCH: [{
  # 2017 Civic Hatchback EX, 2019 Civic Sedan Touring Canadian, and 2018 Civic Hatchback Executive Premium 1.0L CVT European
    57: 3, 148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 427: 3, 428: 8, 432: 7, 441: 5, 450: 8, 460: 3, 464: 8, 470: 2, 476: 7, 477: 8, 479: 8, 490: 8, 493: 5, 495: 8, 506: 8, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 829: 5, 862: 8, 884: 8, 891: 8, 892: 8, 927: 8, 929: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1108: 8, 1302: 8, 1322: 5, 1361: 5, 1365: 5, 1424: 5, 1600: 5, 1601: 8, 1625: 5, 1629: 5, 1633: 8,
  },
  # 2017 Civic Hatchback LX
  {
    57: 3, 148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 423: 2, 427: 3, 428: 8, 432: 7, 441: 5, 450: 8, 464: 8, 470: 2, 476: 7, 477: 8, 479: 8, 490: 8, 493: 5, 495: 8, 506: 8, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 815: 8, 825: 4, 829: 5, 846: 8, 862: 8, 881: 8, 882: 4, 884: 8, 888: 8, 891: 8, 892: 8, 918: 7, 927: 8, 929: 8, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1092: 1, 1108: 8, 1125: 8, 1127: 2, 1296: 8, 1302: 8, 1322: 5, 1361: 5, 1365: 5, 1424: 5, 1600: 5, 1601: 8, 1633: 8
  }],
  CAR.CIVIC_BOSCH_DIESEL: [{
  # 2019 Civic Sedan 1.6 i-dtec Diesel European
    57: 3, 148: 8, 228: 5, 308: 5, 316: 8, 330: 8, 344: 8, 380: 8, 399: 7, 419: 8, 420: 8, 426: 8, 427: 3, 432: 7, 441: 5, 450: 8, 464: 8, 470: 2, 476: 7, 477: 8, 479: 8, 490: 8, 493: 5, 495: 8, 506: 8, 507: 1, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 801: 3, 804: 8, 806: 8, 808: 8, 815: 8, 824: 8, 825: 4, 829: 5, 837: 5, 862: 8, 881: 8, 882: 4, 884: 8, 887: 8, 888: 8, 891: 8, 902: 8, 918: 7, 927: 8, 929: 8, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1092: 1, 1108: 8, 1115: 2, 1125: 8, 1296: 8, 1302: 8, 1322: 5, 1337: 5, 1361: 5, 1365: 5, 1424: 5, 1600: 5, 1601: 8
  }],
  CAR.CRV: [{
    57: 3, 145: 8, 316: 8, 340: 8, 342: 6, 344: 8, 380: 8, 398: 3, 399: 6, 401: 8, 404: 4, 420: 8, 422: 8, 426: 8, 432: 7, 464: 8, 474: 5, 476: 4, 487: 4, 490: 8, 493: 3, 506: 8, 507: 1, 512: 6, 513: 6, 542: 7, 545: 4, 597: 8, 660: 8, 661: 4, 773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 829: 5, 882: 2, 884: 7, 888: 8, 891: 8, 892: 8, 923: 2, 929: 8, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1033: 5, 1036: 8, 1039: 8, 1057: 5, 1064: 7, 1108: 8, 1125: 8, 1296: 8, 1365: 5, 1424: 5, 1600: 5, 1601: 8,
  }],
  CAR.CRV_5G: [{
    57: 3, 148: 8, 199: 4, 228: 5, 231: 5, 232: 7, 304: 8, 330: 8, 340: 8, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 423: 2, 427: 3, 428: 8, 432: 7, 441: 5, 446: 3, 450: 8, 464: 8, 467: 2, 469: 3, 470: 2, 474: 8, 476: 7, 477: 8, 479: 8, 490: 8, 493: 5, 495: 8, 507: 1, 545: 6, 597: 8, 661: 4, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 814: 4, 815: 8, 817: 4, 825: 4, 829: 5, 862: 8, 881: 8, 882: 4, 884: 8, 888: 8, 891: 8, 927: 8, 918: 7, 929: 8, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1108: 8, 1092: 1, 1115: 2, 1125: 8, 1127: 2, 1296: 8, 1302: 8, 1322: 5, 1361: 5, 1365: 5, 1424: 5, 1600: 5, 1601: 8, 1618: 5, 1633: 8, 1670: 5
  }],
  # 1057: 5 1024: 5 are also on the OBD2 bus. their lengths differ from the camera's f-can bus. re-fingerprint after obd2 connection is split in panda firmware from bus 1.
  CAR.CRV_EU: [{
    57: 3, 145: 8, 308: 5, 316: 8, 342: 6, 344: 8, 380: 8, 398: 3, 399: 6, 404: 4, 419: 8, 420: 8, 422: 8, 426: 8, 432: 7, 464: 8, 474: 5, 476: 4, 487: 4, 490: 8, 493: 3, 506: 8, 507: 1, 510: 3, 538: 3, 542: 7, 545: 4, 597: 8, 660: 8, 661: 4, 768: 8, 769: 8, 773: 7, 777: 8, 780: 8, 800: 8, 801: 3, 803: 8, 804: 8, 808: 8, 824: 8, 829: 5, 837: 5, 862: 8, 882: 2, 884: 7, 888: 8, 891: 8, 892: 8, 923: 2, 927: 8, 929: 8, 930: 8, 931: 8, 983: 8, 1024: 8, 1027: 5, 1029: 8, 1033: 5, 1036: 8, 1039: 8, 1040: 8, 1041: 8, 1042: 8, 1043: 8, 1044: 8, 1045: 8, 1046: 8, 1047: 8, 1056: 8, 1057: 8, 1058: 8, 1059: 8, 1060: 8, 1064: 7, 1072: 8, 1073: 8, 1074: 8, 1075: 8, 1076: 8, 1077: 8, 1078: 8, 1079: 8, 1080: 8, 1081: 8, 1088: 8, 1089: 8, 1090: 8, 1091: 8, 1092: 8, 1093: 8, 1108: 8, 1125: 8, 1279: 8, 1280: 8, 1296: 8, 1297: 8, 1365: 5, 1424: 5, 1600: 5, 1601: 8,
  }],
  CAR.CRV_HYBRID: [{
    57: 3, 148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 387: 8, 388: 8, 399: 7, 408: 6, 415: 6, 419: 8, 420: 8, 427: 3, 428: 8, 432: 7, 441: 5, 450: 8, 464: 8, 477: 8, 479: 8, 490: 8, 495: 8, 525: 8, 531: 8, 545: 6, 662: 4, 773: 7, 777: 8, 780: 8, 804: 8, 806: 8, 808: 8, 814: 4, 829: 5, 833: 6, 862: 8, 884: 8, 891: 8, 927: 8, 929: 8, 930: 8, 931: 8, 1302: 8, 1361: 5, 1365: 5, 1600: 5, 1601: 8, 1626: 5, 1627: 5
  }],
  CAR.FIT: [{
    57: 3, 145: 8, 228: 5, 304: 8, 342: 6, 344: 8, 380: 8, 399: 7, 401: 8, 420: 8, 422: 8, 427: 3, 428: 8, 432: 7, 464: 8, 487: 4, 490: 8, 506: 8, 597: 8, 660: 8, 661: 4, 773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 829: 5, 862: 8, 884: 7, 892: 8, 929: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1108: 8, 1322: 5, 1361: 5, 1365: 5, 1424: 5, 1600: 5, 1601: 8
  }],
  # 2018 Odyssey w/ Added Comma Pedal Support (512L & 513L)
  CAR.ODYSSEY: [{
    57: 3, 148: 8, 228: 5, 229: 4, 316: 8, 342: 6, 344: 8, 380: 8, 399: 7, 411: 5, 419: 8, 420: 8, 427: 3, 432: 7, 450: 8, 463: 8, 464: 8, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 542: 7, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 817: 4, 819: 7, 821: 5, 825: 4, 829: 5, 837: 5, 856: 7, 862: 8, 871: 8, 881: 8, 882: 4, 884: 8, 891: 8, 892: 8, 905: 8, 923: 2, 927: 8, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1029: 8, 1036: 8, 1052: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1110: 8, 1125: 8, 1296: 8, 1302: 8, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1614: 5, 1615: 8, 1616: 5, 1619: 5, 1623: 5, 1668: 5
  },
  # 2018 Odyssey Elite w/ Added Comma Pedal Support (512L & 513L)
  {
    57: 3, 148: 8, 228: 5, 229: 4, 304: 8, 342: 6, 344: 8, 380: 8, 399: 7, 411: 5, 419: 8, 420: 8, 427: 3, 432: 7, 440: 8, 450: 8, 463: 8, 464: 8, 476: 4, 490: 8, 506: 8, 507: 1, 542: 7, 545: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 806: 8, 808: 8, 817: 4, 819: 7, 821: 5, 825: 4, 829: 5, 837: 5, 856: 7, 862: 8, 871: 8, 881: 8, 882: 4, 884: 8, 891: 8, 892: 8, 905: 8, 923: 2, 927: 8, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1029: 8, 1036: 8, 1052: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1110: 8, 1125: 8, 1296: 8, 1302: 8, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1614: 5, 1616: 5, 1619: 5, 1623: 5, 1668: 5
  }],
  CAR.ODYSSEY_CHN: [{
    57: 3, 145: 8, 316: 8, 342: 6, 344: 8, 380: 8, 398: 3, 399: 7, 401: 8, 404: 4, 411: 5, 420: 8, 422: 8, 423: 2, 426: 8, 432: 7, 450: 8, 464: 8, 490: 8, 506: 8, 507: 1, 512: 6, 513: 6, 597: 8, 610: 8, 611: 8, 612: 8, 617: 8, 660: 8, 661: 4, 773: 7, 780: 8, 804: 8, 808: 8, 829: 5, 862: 8, 884: 7, 892: 8, 923: 2, 929: 8, 1030: 5, 1137: 8, 1302: 8, 1348: 5, 1361: 5, 1365: 5, 1600: 5, 1601: 8, 1639: 8
  }],
  # 2017 Pilot Touring AND 2016 Pilot EX-L w/ Added Comma Pedal Support (512L & 513L)
  CAR.PILOT: [{
    57: 3, 145: 8, 228: 5, 229: 4, 308: 5, 316: 8, 334: 8, 339: 7, 342: 6, 344: 8, 379: 8, 380: 8, 392: 6, 399: 7, 419: 8, 420: 8, 422: 8, 425: 8, 426: 8, 427: 3, 432: 7, 463: 8, 464: 8, 476: 4, 490: 8, 506: 8, 507: 1, 512: 6, 513: 6, 538: 3, 542: 7, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 829: 5, 837: 5, 856: 7, 871: 8, 882: 2, 884: 7, 891: 8, 892: 8, 923: 2, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1088: 8, 1089: 8, 1108: 8, 1125: 8, 1296: 8, 1424: 5, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1616: 5, 1618: 5, 1668: 5
  }],
  # this fingerprint also includes the Passport 2019
  CAR.PILOT_2019: [{
    57: 3, 145: 8, 228: 5, 308: 5, 316: 8, 334: 8, 342: 6, 344: 8, 379: 8, 380: 8, 399: 7, 411: 5, 419: 8, 420: 8, 422: 8, 425: 8, 426: 8, 427: 3, 432: 7, 463: 8, 464: 8, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 538: 3, 542: 7, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 808: 8, 817: 4, 819: 7, 821: 5, 825: 4, 829: 5, 837: 5, 856: 7, 871: 8, 881: 8, 882: 2, 884: 7, 891: 8, 892: 8, 923: 2, 927: 8, 929: 8, 983: 8, 985: 3, 1029: 8, 1052: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1110: 8, 1125: 8, 1296: 8, 1424: 5, 1445: 8, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1614: 5, 1615: 8, 1616: 5, 1617: 8, 1618: 5, 1623: 5, 1668: 5
  },
  # 2019 Pilot EX-L
  {
    57: 3, 145: 8, 228: 5, 229: 4, 308: 5, 316: 8, 339: 7, 342: 6, 344: 8, 380: 8, 392: 6, 399: 7, 411: 5, 419: 8, 420: 8, 422: 8, 425: 8, 426: 8, 427: 3, 432: 7, 464: 8, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 542: 7, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 808: 8, 817: 4, 819: 7, 821: 5, 829: 5, 871: 8, 881: 8, 882: 2, 884: 7, 891: 8, 892: 8, 923: 2, 927: 8, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1027: 5, 1029: 8, 1039: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1125: 8, 1296: 8, 1424: 5, 1445: 8, 1600: 5, 1601: 8, 1612: 5, 1613: 5, 1616: 5, 1617: 8, 1618: 5, 1623: 5, 1668: 5
  }],
  # Ridgeline w/ Added Comma Pedal Support (512L & 513L)
  CAR.RIDGELINE: [{
    57: 3, 145: 8, 228: 5, 229: 4, 308: 5, 316: 8, 339: 7, 342: 6, 344: 8, 380: 8, 392: 6, 399: 7, 419: 8, 420: 8, 422: 8, 425: 8, 426: 8, 427: 3, 432: 7, 464: 8, 471: 3, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 795: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 829: 5, 871: 8, 882: 2, 884: 7, 892: 8, 923: 2, 927: 8, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1088: 8, 1089: 8, 1108: 8, 1125: 8, 1296: 8, 1365: 5, 1424: 5, 1600: 5, 1601: 8, 1613: 5, 1616: 5, 1618: 5, 1668: 5, 2015: 3
  },
  # 2019 Ridgeline
  {
    57: 3, 145: 8, 228: 5, 229: 4, 308: 5, 316: 8, 339: 7, 342: 6, 344: 8, 380: 8, 392: 6, 399: 7, 419: 8, 420: 8, 422:8, 425: 8, 426: 8, 427: 3, 432: 7, 464: 8, 476: 4, 490: 8, 545: 5, 546: 3, 597: 8, 660: 8, 773: 7, 777: 8, 795: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 871: 8, 882: 2, 884: 7, 892: 8, 923: 2, 929: 8, 963: 8, 965: 8, 966: 8, 967: 8, 983: 8, 985: 3, 1027: 5, 1029: 8, 1036: 8, 1039: 8, 1064: 7, 1088: 8, 1089: 8, 1092: 1, 1108: 8, 1125: 8, 1296: 8, 1365: 5, 424: 5, 1613: 5, 1616: 5, 1618: 5, 1623: 5, 1668: 5
  }],
  # 2019 Insight
  CAR.INSIGHT: [{
    57: 3, 148: 8, 228: 5, 304: 8, 330: 8, 344: 8, 380: 8, 387: 8, 388: 8, 399: 7, 419: 8, 420: 8, 427: 3, 432: 7, 441: 5, 450: 8, 464: 8, 476: 8, 477: 8, 479: 8, 490: 8, 495: 8, 507: 1, 525: 8, 531: 8, 545: 6, 547: 6, 597: 8, 662: 4, 773: 7, 777: 8, 780: 8, 795: 8, 804: 8, 806: 8, 808: 8, 814: 4, 815: 8, 829: 5, 832: 3, 862: 8, 884: 8, 891: 8, 927: 8, 929: 8, 954: 2, 985: 3, 1029: 8, 1093: 4, 1115: 2, 1302: 8, 1361: 5, 1365: 5, 1600: 5, 1601: 8, 1652: 8, 2015: 3
  }]
}

# Don't use theses fingerprints for fingerprinting, they are still needed for ECU detection
IGNORED_FINGERPRINTS = [CAR.INSIGHT, CAR.CIVIC_BOSCH_DIESEL, CAR.CRV_EU]

# add DIAG_MSGS to fingerprints
for c in FINGERPRINTS:
  for f, _ in enumerate(FINGERPRINTS[c]):
    for d in DIAG_MSGS:
      FINGERPRINTS[c][f][d] = DIAG_MSGS[d]

# TODO: Figure out what is relevant
FW_VERSIONS = {
  CAR.ACCORD: {
    (Ecu.programmedFuelInjection, 0x18da10f1, None): [
      b'37805-6A0-A640\x00\x00',
      b'37805-6B2-A550\x00\x00',
      b'37805-6B2-A650\x00\x00',
      b'37805-6B2-A660\x00\x00',
      b'37805-6B2-M520\x00\x00',
    ],
    (Ecu.shiftByWire, 0x18da0bf1, None): [
      b'54008-TVC-A910\x00\x00',
    ],
    (Ecu.transmission, 0x18da1ef1, None): [
      b'28102-6B8-A560\x00\x00',
      b'28102-6B8-M520\x00\x00',
    ],
    (Ecu.electricBrakeBooster, 0x18da2bf1, None): [
      b'46114-TVA-A060\x00\x00',
      b'46114-TVA-A080\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TVA-C050\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TVA-A150\x00\x00',
      b'39990-TVA-A160\x00\x00',
      b'39990-TVA-X030\x00\x00',
    ],
    (Ecu.unknown, 0x18da3af1, None): [
      b'39390-TVA-A020\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TVA-A460\x00\x00',
      b'77959-TVA-X330\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-TVA-A210\x00\x00',
      b'78109-TVC-A010\x00\x00',
      b'78109-TVC-A110\x00\x00',
      b'78109-TVC-A210\x00\x00',
      b'78109-TVC-M510\x00\x00',
    ],
    (Ecu.hud, 0x18da61f1, None): [
      b'78209-TVA-A010\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36802-TVA-A160\x00\x00',
      b'36802-TVA-A160\x00\x00',
      b'36802-TVA-A170\x00\x00',
      b'36802-TWA-A070\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab5f1, None): [
      b'36161-TVA-A060\x00\x00',
      b'36161-TWA-A070\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TVA-A010\x00\x00',
    ],
  },
  CAR.ACCORD_15: {
    (Ecu.programmedFuelInjection, 0x18da10f1, None): [
      b'37805-6A0-9620\x00\x00',
      b'37805-6A0-A640\x00\x00',
      b'37805-6A0-A740\x00\x00',
      b'37805-6A0-A840\x00\x00',
      b'37805-6A0-A850\x00\x00',
    ],
    (Ecu.transmission, 0x18da1ef1, None): [
      b'28101-6A7-A220\x00\x00',
      b'28101-6A7-A320\x00\x00',
      b'28101-6A7-A510\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TVA-A230\x00\x00',
    ],
    (Ecu.electricBrakeBooster, 0x18da2bf1, None): [
      b'46114-TVA-A050\x00\x00',
      b'46114-TVA-A060\x00\x00',
      b'46114-TVA-A120\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-TVA-A010\x00\x00',
      b'78109-TVA-A210\x00\x00',
      b'78109-TVA-A220\x00\x00',
      b'78109-TVA-A310\x00\x00',
      b'78109-TWA-A210\x00\x00',
    ],
    (Ecu.hud, 0x18da61f1, None): [
      b'78209-TVA-A010\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab5f1, None): [
      b'36161-TVA-A060\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TVA-A460\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TVA-B050\x00\x00',
      b'57114-TVA-B040\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36802-TVA-A150\x00\x00',
      b'36802-TVA-A160\x00\x00',
      b'36802-TVA-A170\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TVA-A140\x00\x00',
      b'39990-TVA-A150\x00\x00',  # Are these two different steerRatio?
      b'39990-TVA-A160\x00\x00',  # Sport, Sport 2.0T and Touring 2.0T have different ratios
    ],
  },
  CAR.ACCORDH: {
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TWA-A120\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TWA-A040\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TWA-A440\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-TWA-A010\x00\x00',
      b'78109-TWA-A120\x00\x00',
      b'78109-TWA-A210\x00\x00',
      b'78109-TWA-A110\x00\x00',
    ],
    (Ecu.shiftByWire, 0x18da0bf1, None): [
      b'54008-TWA-A910\x00\x00',
    ],
    (Ecu.hud, 0x18da61f1, None): [
      b'78209-TVA-A010\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab5f1, None): [
      b'36161-TWA-A070\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36802-TWA-A080\x00\x00',
      b'36802-TWA-A070\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TVA-A160\x00\x00',
      b'39990-TVA-A150\x00\x00',
    ],
  },
  CAR.CIVIC: {
    (Ecu.programmedFuelInjection, 0x18da10f1, None): [
      b'37805-5AA-A640\x00\x00',
      b'37805-5AA-A650\x00\x00',
      b'37805-5AA-A670\x00\x00',
      b'37805-5AA-A680\x00\x00',
      b'37805-5AA-A810\x00\x00',
      b'37805-5AA-C820\x00\x00',
      b'37805-5AA-L660\x00\x00',
      b'37805-5AJ-A610\x00\x00',
      b'37805-5BA-A510\x00\x00',
      b'37805-5BA-L940\x00\x00',
      b'37805-5BA-L960\x00\x00',
    ],
    (Ecu.transmission, 0x18da1ef1, None): [
      b'28101-5CG-A040\x00\x00',
      b'28101-5CG-A050\x00\x00',
      b'28101-5CG-A070\x00\x00',
      b'28101-5CG-A080\x00\x00',
      b'28101-5CG-A810\x00\x00',
      b'28101-5CG-A820\x00\x00',
      b'28101-5DJ-A040\x00\x00',
      b'28101-5DJ-A060\x00\x00',
      b'28101-5DJ-A510\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TBA-A550\x00\x00',
      b'57114-TBA-A560\x00\x00',
      b'57114-TBA-A570\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TBA,A030\x00\x00',
      b'39990-TBA-A030\x00\x00',
      b'39990-TBG-A030\x00\x00',
      b'39990-TEG-A010\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TBA-A030\x00\x00',
      b'77959-TBA-A040\x00\x00',
      b'77959-TBG-A030\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-TBC-A310\x00\x00',
      b'78109-TBC-A320\x00\x00',
      b'78109-TBC-A510\x00\x00',
      b'78109-TBC-A520\x00\x00',
      b'78109-TBC-A530\x00\x00',
      b'78109-TBC-C530\x00\x00',
      b'78109-TBH-A530\x00\x00',
      b'78109-TEG-A310\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab0f1, None): [
      b'36161-TBA-A030\x00\x00',
      b'36161-TBC-A020\x00\x00',
      b'36161-TBC-A030\x00\x00',
      b'36161-TEG-A010\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TBA-A010\x00\x00',
      b'38897-TBA-A020\x00\x00',
    ],
  },
  CAR.CIVIC_BOSCH: {
    (Ecu.programmedFuelInjection, 0x18da10f1, None): [
      b'37805-5AA-A950\x00\x00',
      b'37805-5AA-L950\x00\x00',
      b'37805-5AN-A750\x00\x00',
      b'37805-5AN-A830\x00\x00',
      b'37805-5AN-A930\x00\x00',
      b'37805-5AN-A950\x00\x00',
      b'37805-5AN-L940\x00\x00',
      b'37805-5AN-LH20\x00\x00',
      b'37805-5AN-LJ20\x00\x00',
      b'37805-5AZ-E850\x00\x00',
      b'37805-5BB-L640\x00\x00',
      b'37805-5AN-AH20\x00\x00',
    ],
    (Ecu.transmission, 0x18da1ef1, None): [
      b'28101-5CG-A920\x00\x00',
      b'28101-5CG-C220\x00\x00',
      b'28101-5CG-C320\x00\x00',
      b'28101-5CK-A130\x00\x00',
      b'28101-5CK-A140\x00\x00',
      b'28101-5CK-A150\x00\x00',
      b'28101-5CK-C130\x00\x00',
      b'28101-5DJ-A710\x00\x00',
      b'28101-5DV-E330\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TBG-A340\x00\x00',
      b'57114-TGG-A340\x00\x00',
      b'57114-TGL-G330\x00\x00',
      b'57114-TGG-C320\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TBA-C020\x00\x00',
      b'39990-TBA-C120\x00\x00',
      b'39990-TGG-A020\x00\x00',
      b'39990-TGG-A120\x00\x00',
      b'39990-TGL-E130\x00\x00',
      b'39990-TGG-A020\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TBA-A060\x00\x00',
      b'77959-TGG-A020\x00\x00',
      b'77959-TGG-G010\x00\x00',
      b'77959-TGG-A020\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-TBA-A910\x00\x00',
      b'78109-TBC-A740\x00\x00',
      b'78109-TGG-A210\x00\x00',
      b'78109-TGG-A310\x00\x00',
      b'78109-TGG-A320\x00\x00',
      b'78109-TGG-A810\x00\x00',
      b'78109-TGG-A820\x00\x00',
      b'78109-TGL-G120\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36802-TBA-A150\x00\x00',
      b'36802-TGG-A050\x00\x00',
      b'36802-TGL-G040\x00\x00',
      b'36802-TGG-A060\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab5f1, None): [
      b'36161-TBA-A130\x00\x00',
      b'36161-TGG-A060\x00\x00',
      b'36161-TGL-G050\x00\x00',
      b'36161-TGG-A080\x00\x00',
    ],
    (Ecu.unknown, 0x18daeff1, None): [
      b'38897-TBA-A110\x00\x00',
      b'38897-TBA-A020\x00\x00',
      b'38897-TBA-A020\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TBA-A110\x00\x00',
      b'38897-TBA-A020\x00\x00',
    ],
  },
  CAR.CIVIC_BOSCH_DIESEL: {
    (Ecu.programmedFuelInjection, 0x18da10f1, None): [b'37805-59N-G830\x00\x00'],
    (Ecu.transmission, 0x18da1ef1, None): [b'28101-59Y-G620\x00\x00'],
    (Ecu.vsa, 0x18da28f1, None): [b'57114-TGN-E320\x00\x00'],
    (Ecu.eps, 0x18da30f1, None): [b'39990-TFK-G020\x00\x00'],
    (Ecu.srs, 0x18da53f1, None): [b'77959-TFK-G210\x00\x00'],
    (Ecu.combinationMeter, 0x18da60f1, None): [b'78109-TFK-G020\x00\x00'],
    (Ecu.fwdRadar, 0x18dab0f1, None): [b'36802-TFK-G130\x00\x00'],
    (Ecu.shiftByWire, 0x18da0bf1, None): [b'54008-TGN-E010\x00\x00'],
  },
  CAR.CRV_5G: {
    (Ecu.programmedFuelInjection, 0x18da10f1, None): [
      b'37805-5PA-3080\x00\x00',
      b'37805-5PA-4050\x00\x00',
      b'37805-5PA-6530\x00\x00',
      b'37805-5PA-6630\x00\x00',
      b'37805-5PA-A670\x00\x00',
      b'37805-5PA-A680\x00\x00',
      b'37805-5PA-A850\x00\x00',
      b'37805-5PA-A870\x00\x00',
      b'37805-5PA-A880\x00\x00',
      b'37805-5PA-A890\x00\x00',
    ],
    (Ecu.transmission, 0x18da1ef1, None): [
      b'28101-5RG-A020\x00\x00',
      b'28101-5RG-A030\x00\x00',
      b'28101-5RG-A040\x00\x00',
      b'28101-5RG-A120\x00\x00',
      b'28101-5RH-A030\x00\x00',
      b'28101-5RH-A040\x00\x00',
      b'28101-5RH-A120\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TLA-A040\x00\x00',
      b'57114-TLA-A050\x00\x00',
      b'57114-TLA-A060\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TLA-A040\x00\x00',
      b'39990-TLA,A040\x00\x00',
    ],
    (Ecu.electricBrakeBooster, 0x18da2bf1, None): [
      b'46114-TLA-A040\x00\x00',
      b'46114-TLA-A050\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-TLA-A110\x00\x00',
      b'78109-TLA-A210\x00\x00',
      b'78109-TLA-C210\x00\x00',
      b'78109-TLB-A110\x00\x00',
      b'78109-TLB-A210\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TLA-A010\x00\x00',
      b'38897-TNY-G010\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36802-TLA-A040\x00\x00',
      b'36802-TLA-A050\x00\x00',
      b'36802-TLA-A060\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab5f1, None): [
      b'36161-TLA-A060\x00\x00',
      b'36161-TLA-A070\x00\x00',
      b'36161-TLA-A080\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TLA-A240\x00\x00',
      b'77959-TLA-A250\x00\x00',
      b'77959-TLA-A320\x00\x00',
    ],
  },
  CAR.CRV_EU: {
    (Ecu.programmedFuelInjection, 0x18da10f1, None): [b'37805-R5Z-G740\x00\x00'],
    (Ecu.vsa, 0x18da28f1, None): [b'57114-T1V-G920\x00\x00'],
    (Ecu.fwdRadar, 0x18dab0f1, None): [b'36161-T1V-G520\x00\x00'],
    (Ecu.shiftByWire, 0x18da0bf1, None): [b'54008-T1V-G010\x00\x00'],
    (Ecu.transmission, 0x18da1ef1, None): [b'28101-5LH-E120\x00\x00'],
    (Ecu.combinationMeter, 0x18da60f1, None): [b'78109-T1V-G020\x00\x00'],
    (Ecu.srs, 0x18da53f1, None): [b'77959-T1G-G940\x00\x00'],
  },
  CAR.CRV_HYBRID: {
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TPA-G020\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TPA-G030\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TMA-H110\x00\x00',
    ],
    (Ecu.shiftByWire, 0x18da0bf1, None): [
      b'54008-TMB-H510\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab5f1, None): [
      b'36161-TPA-E050\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-TPA-G520\x00\x00',
    ],
    (Ecu.hud, 0x18da61f1, None): [
      b'78209-TLA-X010\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36802-TPA-E040\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TLA-G220\x00\x00',
    ],
  },
  CAR.ODYSSEY: {
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-THR-A010\x00\x00',
      b'38897-THR-A020\x00\x00',
    ],
    (Ecu.programmedFuelInjection, 0x18da10f1, None): [
      b'37805-5MR-A250\x00\x00',
      b'37805-5MR-A310\x00\x00',
      b'37805-5MR-A750\x00\x00',
      b'37805-5MR-A840\x00\x00',
      b'37805-5MR-C620\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-THR-A020\x00\x00',
      b'39990-THR-A030\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-THR-A010\x00\x00',
      b'77959-THR-A110\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab0f1, None): [
      b'36161-THR-A030\x00\x00',
      b'36161-THR-A110\x00\x00',
      b'36161-THR-A720\x00\x00',
      b'36161-THR-A810\x00\x00',
      b'36161-THR-C010\x00\x00',
    ],
    (Ecu.transmission, 0x18da1ef1, None): [
      b'28101-5NZ-A310\x00\x00',
      b'28101-5NZ-C310\x00\x00',
      b'28102-5MX-A001\x00\x00',
      b'28102-5MX-A610\x00\x00',
      b'28102-5MX-A710\x00\x00',
      b'28102-5MX-A900\x00\x00',
      b'28102-5MX-A910\x00\x00',
      b'28102-5MX-C001\x00\x00',
      b'28103-5NZ-A300\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-THR-A040\x00\x00',
      b'57114-THR-A110\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-THR-A230\x00\x00',
      b'78109-THR-A430\x00\x00',
      b'78109-THR-A820\x00\x00',
      b'78109-THR-AB20\x00\x00',
      b'78109-THR-AB20\x00\x00',
      b'78109-THR-AB40\x00\x00',
      b'78109-THR-AC40\x00\x00',
      b'78109-THR-AE40\x00\x00',
      b'78109-THR-AL10\x00\x00',
      b'78109-THR-C330\x00\x00',
      b'78109-THR-CE20\x00\x00',
    ],
    (Ecu.shiftByWire, 0x18da0bf1, None): [
      b'54008-THR-A020\x00\x00',
    ],
  },
  CAR.PILOT_2019: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TG7-A060\x00\x00',
      b'39990-TGS-A230\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TG7-A110\x00\x00',
      b'38897-TG7-A030\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab0f1, None): [
      b'36161-TG7-A630\x00\x00',
      b'36161-TG7-A930\x00\x00',
      b'36161-TG8-A630\x00\x00',
      b'36161-TGS-A130\x00\x00',
      b'36161-TGT-A030\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TG7-A210\x00\x00',
      b'77959-TGS-A010\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-TG7-AJ20\x00\x00',
      b'78109-TG7-AP10\x00\x00',
      b'78109-TG7-AP20\x00\x00',
      b'78109-TG8-AJ20\x00\x00',
      b'78109-TGS-AK20\x00\x00',
      b'78109-TGS-AP20\x00\x00',
      b'78109-TGT-AJ20\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TG7-A630\x00\x00',
      b'57114-TG7-A730\x00\x00',
      b'57114-TG8-A630\x00\x00',
      b'57114-TGS-A530\x00\x00',
      b'57114-TGT-A530\x00\x00',
    ],
  },
  CAR.RIDGELINE: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-T6Z-A020\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab0f1, None): [
      b'36161-T6Z-A310\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-T6Z-A010\x00\x00',
    ],
    (Ecu.combinationMeter, 0x18da60f1, None): [
      b'78109-T6Z-A420\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-T6Z-A020\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-T6Z-A130\x00\x00',
    ],
  },
  CAR.INSIGHT: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TXM-A040\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36802-TXM-A070\x00\x00',
    ],
    (Ecu.fwdCamera, 0x18dab5f1, None): [
      b'36161-TXM-A050\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TXM-A230\x00\x00',
    ],
  },
}

DBC = {
  CAR.ACCORD: dbc_dict('honda_accord_s2t_2018_can_generated', None),
  CAR.ACCORD_15: dbc_dict('honda_accord_lx15t_2018_can_generated', None),
  CAR.ACCORDH: dbc_dict('honda_accord_s2t_2018_can_generated', None),
  CAR.ACURA_ILX: dbc_dict('acura_ilx_2016_can_generated', 'acura_ilx_2016_nidec'),
  CAR.ACURA_RDX: dbc_dict('acura_rdx_2018_can_generated', 'acura_ilx_2016_nidec'),
  CAR.CIVIC: dbc_dict('honda_civic_touring_2016_can_generated', 'acura_ilx_2016_nidec'),
  CAR.CIVIC_BOSCH: dbc_dict('honda_civic_hatchback_ex_2017_can_generated', None),
  CAR.CIVIC_BOSCH_DIESEL: dbc_dict('honda_civic_sedan_16_diesel_2019_can_generated', None),
  CAR.CRV: dbc_dict('honda_crv_touring_2016_can_generated', 'acura_ilx_2016_nidec'),
  CAR.CRV_5G: dbc_dict('honda_crv_ex_2017_can_generated', None),
  CAR.CRV_EU: dbc_dict('honda_crv_executive_2016_can_generated', 'acura_ilx_2016_nidec'),
  CAR.CRV_HYBRID: dbc_dict('honda_crv_hybrid_2019_can_generated', None),
  CAR.FIT: dbc_dict('honda_fit_ex_2018_can_generated', 'acura_ilx_2016_nidec'),
  CAR.ODYSSEY: dbc_dict('honda_odyssey_exl_2018_generated', 'acura_ilx_2016_nidec'),
  CAR.ODYSSEY_CHN: dbc_dict('honda_odyssey_extreme_edition_2018_china_can_generated', 'acura_ilx_2016_nidec'),
  CAR.PILOT: dbc_dict('honda_pilot_touring_2017_can_generated', 'acura_ilx_2016_nidec'),
  CAR.PILOT_2019: dbc_dict('honda_pilot_touring_2017_can_generated', 'acura_ilx_2016_nidec'),
  CAR.RIDGELINE: dbc_dict('honda_ridgeline_black_edition_2017_can_generated', 'acura_ilx_2016_nidec'),
  CAR.INSIGHT: dbc_dict('honda_insight_ex_2019_can_generated', None),
}

STEER_THRESHOLD = {
  CAR.ACCORD: 1200,
  CAR.ACCORD_15: 1200,
  CAR.ACCORDH: 1200,
  CAR.ACURA_ILX: 1200,
  CAR.ACURA_RDX: 400,
  CAR.CIVIC: 1200,
  CAR.CIVIC_BOSCH: 1200,
  CAR.CIVIC_BOSCH_DIESEL: 1200,
  CAR.CRV: 1200,
  CAR.CRV_5G: 1200,
  CAR.CRV_EU: 400,
  CAR.CRV_HYBRID: 1200,
  CAR.FIT: 1200,
  CAR.ODYSSEY: 1200,
  CAR.ODYSSEY_CHN: 1200,
  CAR.PILOT: 1200,
  CAR.PILOT_2019: 1200,
  CAR.RIDGELINE: 1200,
  CAR.INSIGHT: 1200,
}

SPEED_FACTOR = {
  CAR.ACCORD: 1.,
  CAR.ACCORD_15: 1.,
  CAR.ACCORDH: 1.,
  CAR.ACURA_ILX: 1.,
  CAR.ACURA_RDX: 1.,
  CAR.CIVIC: 1.,
  CAR.CIVIC_BOSCH: 1.,
  CAR.CIVIC_BOSCH_DIESEL: 1.,
  CAR.CRV: 1.025,
  CAR.CRV_5G: 1.025,
  CAR.CRV_EU: 1.025,
  CAR.CRV_HYBRID: 1.025,
  CAR.FIT: 1.,
  CAR.ODYSSEY: 1.,
  CAR.ODYSSEY_CHN: 1.,
  CAR.PILOT: 1.,
  CAR.PILOT_2019: 1.,
  CAR.RIDGELINE: 1.,
  CAR.INSIGHT: 1.,
}

# msgs sent for steering controller by camera module on can 0.
# those messages are mutually exclusive on CRV and non-CRV cars
ECU_FINGERPRINT = {
  Ecu.fwdCamera: [0xE4, 0x194],   # steer torque cmd
}

HONDA_BOSCH = [CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH, CAR.CIVIC_BOSCH, CAR.CIVIC_BOSCH_DIESEL, CAR.CRV_5G, CAR.CRV_HYBRID, CAR.INSIGHT]
