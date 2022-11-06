#!/usr/bin/env python3

import cereal.messaging as messaging
from selfdrive.car import gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from selfdrive.car.tesla.values import DBC, CAR, CAN_CHASSIS, GEAR_MAP
import crcmod
from cereal import car


targetMaxBraking = 90.0 #psi
targetBrakeHold = 15.0 #psi

crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)
packer = CANPacker(DBC[CP.carFingerprint]['chassis'])

class PedalCalibrator:

  @staticmethod
  def checksum(msg_id, dat):
    # TODO: get message ID from name instead
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def get_parser_can0(self,CP):
    # Status messages
    new_signals = [
          ('DI_torque1','DI_pedalPos'),
          ('DI_torque2','DI_gear'),
          ('BrakeMessage','driverBrakeStatus')
    ]
    checks = [
          ('BrakeMessage',1),
          ('DI_torque1',1),
          ('DI_torque2',1)
    ]
    return CANParser(DBC[CP.carFingerprint]['chassis'], new_signals, checks, CAN_CHASSIS[CP.carFingerprint])

  def get_parser_can2(self,CP):
    # Pedal messages
    new_signals = [
          ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
          ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0),
          ("STATE", "GAS_SENSOR", 0),
          ("IDX", "GAS_SENSOR", 0),
    ]
    checks = [
          ("GAS_SENSOR", 0),
    ]
    return return CANParser(DBC[CP.carFingerprint]['chassis'], new_signals, checks, 2, enforce_checks=False)
    
  def create_pedal_command_msg(self,accelCommand, enable, idx, pedalcan):
    """Create GAS_COMMAND (0x551) message to comma pedal"""
    msg_id = 0x551
    msg_len = 6
    msg = create_string_buffer(msg_len)
    m1 = 0.050796813
    m2 = 0.101593626
    d = -22.85856576
    if enable == 1:
        int_accelCommand = int((accelCommand - d) / m1)
        int_accelCommand2 = int((accelCommand - d) / m2)
    else:
        int_accelCommand = 0
        int_accelCommand2 = 0
    msg = create_string_buffer(msg_len)
    struct.pack_into(
        "BBBBB",
        msg,
        0,
        int((int_accelCommand >> 8) & 0xFF),
        int_accelCommand & 0xFF,
        int((int_accelCommand2 >> 8) & 0xFF),
        int_accelCommand2 & 0xFF,
        ((enable << 7) + idx) & 0xFF,
    )
    struct.pack_into("B", msg, msg_len - 1, checksum(msg_id,msg.raw))
    return [msg_id, 0, msg.raw, pedalcan]

  def __init__(self):
    CP = CarInterfaceBase.get_std_params(CAR.PREAP_MODELS,gen_empty_fingerprint())
    self.cp = self.get_parser_can0(CP)
    self.cp2 = self.get_parser_can2(CP)
    self.can_sock = self.messaging.sub_sock('can')
    self.can_define = CANDefine(DBC[CP.carFingerprint]['chassis'])

    self.last_pedal_idx = -1


  def run(self):
    while 1:
      can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
      cp.update_strings(can_strings)
      cp2.update_strings(can_strings)

      #Gear
      gearShifter = GEAR_MAP[can_define.dv["DI_torque2"]["DI_gear"].get(int(cp.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]
      gear_neutral = gearShifter == car.CarState.GearShifter.neutral

      #DI Pedal Level
      di_gas = cp.vl["DI_torque1"]["DI_pedalPos"] / 100.0

      #Pedal Msg
      pedal_interceptor_state = cp.vl["GAS_SENSOR"]["STATE"]
      pedal_interceptor_value = cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"]
      pedal_interceptor_value2 = cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]
      pedal_idx = cp.vl["GAS_SENSOR"]["IDX"]

      #print(chr(27) + "[2J")
      #print("Checking",message,"...")
      #print("--------------------------------------------")
      #for signal in signals:
      #  val = cp.vl[message][signal]
      #  print(signal,val)


if __name__ == "__main__":