#!/usr/bin/env python3

import cereal.messaging as messaging
from selfdrive.car.tesla.radar_interface import RadarInterface
from selfdrive.car.tesla.values import CAR
from selfdrive.car.interfaces import CarInterfaceBase


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
        print (pt)
