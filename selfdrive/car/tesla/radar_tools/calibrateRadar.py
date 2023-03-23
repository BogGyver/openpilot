#!/usr/bin/env python3

import cereal.messaging as messaging
from selfdrive.car.tesla.radar_interface import RadarInterface
from selfdrive.car.tesla.values import CAR
from selfdrive.car.interfaces import CarInterfaceBase


#for calibration we only want fixed objects within 1 m of the center line and between 2.5 and 4.5 m far from radar
MINX = 2.5
MAXX = 14.5
MINY = -1.0
MAXY = 1.0

if __name__ == "__main__":
  CP = CarInterfaceBase.get_std_params(CAR.PREAP_MODELS)
  CP.radarTimeStep = (1.0 / 8) # 8Hz
  RI = RadarInterface(CP)
  can_sock = messaging.sub_sock('can')
  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    rr = RI.update(can_strings)

    if (rr is None):
      continue

    print(chr(27) + "[2J")

    for pt in rr.points:
      if (pt.dRel >= MINX) and (pt.dRel <= MAXX) and (pt.yRel >= MINY) and (pt.yRel <= MAXY):
        print (pt)
