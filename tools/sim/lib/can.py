#!/usr/bin/env python3
import time
import cereal.messaging as messaging
from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp
from selfdrive.car.honda.values import FINGERPRINTS, CAR
from selfdrive.car import crc8_pedal
import math

from selfdrive.test.longitudinal_maneuvers.plant import get_car_can_parser
cp = get_car_can_parser()
#cp = CANParser("honda_civic_touring_2016_can_generated")

packer = CANPacker("honda_civic_touring_2016_can_generated")
rpacker = CANPacker("acura_ilx_2016_nidec")

SR = 7.5

def angle_to_sangle(angle):
  return - math.degrees(angle) * SR

def can_function(pm, speed, angle, idx, cruise_button=0, is_engaged=False):
  msg = []
  msg.append(packer.make_can_msg("ENGINE_DATA", 0, {"XMISSION_SPEED": speed}, idx))
  msg.append(packer.make_can_msg("WHEEL_SPEEDS", 0,
    {"WHEEL_SPEED_FL": speed,
     "WHEEL_SPEED_FR": speed,
     "WHEEL_SPEED_RL": speed,
     "WHEEL_SPEED_RR": speed}, -1))

  msg.append(packer.make_can_msg("SCM_BUTTONS", 0, {"CRUISE_BUTTONS": cruise_button}, idx))

  values = {"COUNTER_PEDAL": idx&0xF}
  checksum = crc8_pedal(packer.make_can_msg("GAS_SENSOR", 0, {"COUNTER_PEDAL": idx&0xF}, -1)[2][:-1])
  values["CHECKSUM_PEDAL"] = checksum
  msg.append(packer.make_can_msg("GAS_SENSOR", 0, values, -1))

  msg.append(packer.make_can_msg("GEARBOX", 0, {"GEAR": 4, "GEAR_SHIFTER": 8}, idx))
  msg.append(packer.make_can_msg("GAS_PEDAL_2", 0, {}, idx))
  msg.append(packer.make_can_msg("SEATBELT_STATUS", 0, {"SEATBELT_DRIVER_LATCHED": 1}, idx))
  msg.append(packer.make_can_msg("STEER_STATUS", 0, {}, idx))
  msg.append(packer.make_can_msg("STEERING_SENSORS", 0, {"STEER_ANGLE":angle_to_sangle(angle)}, idx))
  msg.append(packer.make_can_msg("POWERTRAIN_DATA", 0, {}, idx))
  msg.append(packer.make_can_msg("VSA_STATUS", 0, {}, idx))
  msg.append(packer.make_can_msg("STANDSTILL", 0, {}, idx))
  msg.append(packer.make_can_msg("STEER_MOTOR_TORQUE", 0, {}, idx))
  msg.append(packer.make_can_msg("EPB_STATUS", 0, {}, idx))
  msg.append(packer.make_can_msg("DOORS_STATUS", 0, {}, idx))
  msg.append(packer.make_can_msg("CRUISE_PARAMS", 0, {}, idx))
  msg.append(packer.make_can_msg("CRUISE", 0, {}, idx))
  msg.append(packer.make_can_msg("SCM_FEEDBACK", 0, {"MAIN_ON": 1}, idx))
  msg.append(packer.make_can_msg("POWERTRAIN_DATA", 0, {"ACC_STATUS": int(is_engaged)}, idx))
  #print(msg)

  # cam bus
  msg.append(packer.make_can_msg("STEERING_CONTROL", 2, {}, idx))
  msg.append(packer.make_can_msg("ACC_HUD", 2, {}, idx))
  msg.append(packer.make_can_msg("BRAKE_COMMAND", 2, {}, idx))

  # radar
  if idx%5 == 0:
    msg.append(rpacker.make_can_msg("RADAR_DIAGNOSTIC", 1, {"RADAR_STATE": 0x79}, -1))
    for i in range(16):
      msg.append(rpacker.make_can_msg("TRACK_%d" % i, 1, {"LONG_DIST": 255.5}, -1))

  # fill in the rest for fingerprint
  done = set([x[0] for x in msg])
  for k,v in FINGERPRINTS[CAR.CIVIC][0].items():
    if k not in done and k not in [0xE4, 0x194]:
      msg.append([k, 0, b'\x00'*v, 0])
  pm.send('can', can_list_to_can_capnp(msg))

def sendcan_function(sendcan):
  sc = messaging.drain_sock_raw(sendcan)
  cp.update_strings(sc, sendcan=True)

  if cp.vl[0x1fa]['COMPUTER_BRAKE_REQUEST']:
    brake = cp.vl[0x1fa]['COMPUTER_BRAKE'] * 0.003906248
  else:
    brake = 0.0 

  if cp.vl[0x200]['GAS_COMMAND'] > 0:
    gas = cp.vl[0x200]['GAS_COMMAND'] / 256.0
  else:
    gas = 0.0 

  if cp.vl[0xe4]['STEER_TORQUE_REQUEST']:
    steer_torque = cp.vl[0xe4]['STEER_TORQUE']*1.0/0x1000
  else:
    steer_torque = 0.0 

  return (gas, brake, steer_torque)

if __name__ == "__main__":
  pm = messaging.PubMaster(['can'])
  sendcan = messaging.sub_sock('sendcan')
  idx = 0
  while 1:
    sendcan_function(sendcan)
    can_function(pm, 10.0, idx)
    time.sleep(0.01)
    idx += 1

