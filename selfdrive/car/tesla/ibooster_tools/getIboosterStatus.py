#!/usr/bin/env python3

import cereal.messaging as messaging
from selfdrive.car import gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from opendbc.can.parser import CANParser
from selfdrive.car.tesla.values import DBC, CAR, CAN_CHASSIS


signals = ["DriverBrakeApplied","BrakePedalPosition",
    "BrakeApplied","BrakeOK","Status",
    "Brake_Counter","Brake_Checksum"]
message = "ECU_BrakeStatus"

def get_parser(CP):
  # Status messages
  new_signals = []
  checks = [('ECU_BrakeStatus', 1),]
  for signal in signals:
    tpl = (signal, message)
    new_signals.append(tpl)
  return CANParser(DBC[CP.carFingerprint]['chassis'], new_signals, checks, CAN_CHASSIS[CP.carFingerprint])
  

if __name__ == "__main__":
  CP = CarInterfaceBase.get_std_params(CAR.PREAP_MODELS,gen_empty_fingerprint())
  rcp = get_parser(CP)
  can_sock = messaging.sub_sock('can')
  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    values = rcp.update_strings(can_strings)
    print(chr(27) + "[2J")
    print("Checking",message,"...")
    print("--------------------------------------------")
    for signal in signals:
      val = rcp.vl[message][signal]
      print(signal,val)


