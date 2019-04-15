#!/usr/bin/env python
from cereal import car
import time
import os
import zmq
from selfdrive.can.parser import CANParser
from common.realtime import sec_since_boot
from selfdrive.services import service_list
import selfdrive.messaging as messaging

RADAR_A_MSGS = list(range(0x310, 0x36F , 3))
RADAR_B_MSGS = list(range(0x311, 0x36F, 3))

def _create_radard_can_parser():
  dbc_f = 'teslaradar.dbc'

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)

  signals = zip(['LongDist'] * msg_a_n + ['NEW_TRACK'] * msg_a_n + ['LatDist'] * msg_a_n +
                ['LongSpeed'] * msg_a_n + ['LongAccel'] * msg_a_n + ['LatSpeed'] * msg_b_n,
                RADAR_A_MSGS * 5 + RADAR_B_MSGS,
                [255] * msg_a_n + [1] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n + [0] * msg_b_n)

  checks = zip(RADAR_A_MSGS + RADAR_B_MSGS, [20]*(msg_a_n + msg_b_n))

  return CANParser(os.path.splitext(dbc_f)[0], signals, checks, 1)


class RadarInterface(object):
  def __init__(self, CP):
    # radar
    self.pts = {}
    self.delay = 0.1

  def update(self):

    ret = car.RadarState.new_message()
    time.sleep(0.05)  # radard runs on RI updates
    
    return ret

if __name__ == "__main__":
  RI = RadarInterface()
  while 1:
    ret = RI.update()
    print(chr(27) + "[2J")
    print ret
