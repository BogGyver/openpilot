#!/usr/bin/env python3

import cereal.messaging as messaging
from selfdrive.car import gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from selfdrive.car.tesla.values import DBC, CAR, CAN_CHASSIS
import crcmod


targetMaxBraking = 90.0 #psi
targetBrakeHold = 15.0 #psi

crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)
packer = CANPacker(DBC[CP.carFingerprint]['chassis'])

def get_parser(CP):
  # Status messages
  new_signals = [('ESP_145h','ESP_brakeMasterCylPress'),
        ('ECU_BrakeStatus','BrakeOK'),
        ('ECU_BrakeStatus','Status'),
        ('BrakeMessage','driverBrakeStatus')]
  checks = [('ECU_BrakeStatus', 1),
        ('BrakeMessage',1),
        ('ESP_145h',1)]
  return CANParser(DBC[CP.carFingerprint]['chassis'], new_signals, checks, CAN_CHASSIS[CP.carFingerprint])
  
def create_ibst_command(brake, counter, bus):
  values = {
    "BrakePositionCommand" : brake,
    "BrakeRelativeCommand": 0,
    "BrakeMode": 2,
    "Brake_Counter" : counter,
    "Brake_Checksum" : 0,
  }
  data = packer.make_can_msg("ECU_BrakeCommand", bus, values)[2]
  values["Brake_Checksum"] = crc(data[1:6])
  return packer.make_can_msg("ECU_BrakeCommand", bus, values)

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


