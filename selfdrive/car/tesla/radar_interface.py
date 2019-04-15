#!/usr/bin/env python
from cereal import car
import time
import os
import zmq
from selfdrive.can.parser import CANParser
from common.realtime import sec_since_boot
from selfdrive.services import service_list
import selfdrive.messaging as messaging
from selfdrive.car.tesla.readconfig import read_config_file,CarSettings

RADAR_A_MSGS = list(range(0x310, 0x36F , 3))
RADAR_B_MSGS = list(range(0x311, 0x36F, 3))

BOSCH_MAX_DIST = 150. #max distance for radar

def _create_radard_can_parser():
  dbc_f = 'teslaradar.dbc'

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)

  signals = zip(['LongDist'] * msg_a_n +  ['LatDist'] * msg_a_n +
                ['LongSpeed'] * msg_a_n + ['LongAccel'] * msg_a_n + 
                ['Flags'] * msg_a_n + ['Flag1'] * msg_a_n + 
                ['Valid'] * msg_a_n + ['Existing'] * msg_a_n + 
                ['Index'] * msg_a_n + ['LatSpeed'] * msg_b_n + ['Index2'] * msg_b_n,
                RADAR_A_MSGS * 9 + RADAR_B_MSGS * 2,
                [255] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n + 
                [0] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n +
                [3] * msg_a_n + [0] * msg_b_n +[3] * msg_b_n)

  checks = zip(RADAR_A_MSGS + RADAR_B_MSGS, [20]*(msg_a_n + msg_b_n))

  return CANParser(os.path.splitext(dbc_f)[0], signals, checks, 1)


class RadarInterface(object):
  def __init__(self, CP):
    # radar
    self.pts = {}
    self.delay = 0.1
    self.useTeslaRadar = CarSettings().get_value("useTeslaRadar")
    if self.useTeslaRadar:
      self.pts = {}
      self.valid_cnt = {key: 0 for key in RADAR_A_MSGS}
      self.track_id = 0
      self.delay = 0.0  # Delay of radar
      self.rcp = _create_radard_can_parser()
      context = zmq.Context()
      self.logcan = messaging.sub_sock(context, service_list['can'].port)

  def update(self):

    ret = car.RadarState.new_message()
    if not self.useTeslaRadar:
      # TODO: make a adas dbc file for dsu-less models
      time.sleep(0.05)
      return ret

    canMonoTimes = []
    updated_messages = set()
    while 1:
      tm = int(sec_since_boot() * 1e9)
      updated_messages.update(self.rcp.update(tm, True))
      if RADAR_B_MSGS[-1] in updated_messages:
        break
    errors = []
    if not self.rcp.can_valid:
      errors.append("commIssue")
    ret.errors = errors
    ret.canMonoTimes = canMonoTimes
    for ii in updated_messages:
      if ii in RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]
        if (cpt['LongDist'] >= BOSCH_MAX_DIST) or (cpt['Valid'] and not cpt['Existing']):
          self.valid_cnt[ii] = 0    # reset counter
        if cpt['Valid'] and cpt['LongDist'] < BOSCH_MAX_DIST:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] -1, 0)

        #score = self.rcp.vl[ii+16]['SCORE']
        print ii, self.valid_cnt[ii], cpt['Valid'], cpt['LongDist'], cpt['LatDist']

        # radar point only valid if it's a valid measurement and score is above 50
        # bosch radar data needs to match Index and Index2 for validity
        if cpt['Valid'] and (cpt['LongDist'] < BOSCH_MAX_DIST) and (self.valid_cnt[ii] > 0) and (cpt['Index'] == self.rcp.vl[ii+1]['Index2']):
          if ii not in self.pts or (cpt['Valid'] and not cpt['Existing']):
            self.pts[ii] = car.RadarState.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LongDist']  # from front of car
          self.pts[ii].yRel = -cpt['LatDist']  # in car frame's y axis, left is positive
          self.pts[ii].vRel = cpt['LongSpeed']
          self.pts[ii].aRel = cpt['LongAccel']
          self.pts[ii].yvRel = self.rcp.vl[ii+1]['LatSpeed']
          self.pts[ii].measured = bool(cpt['Valid'])
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = self.pts.values()
    return ret



if __name__ == "__main__":
  RI = RadarInterface()
  while 1:
    ret = RI.update()
    print(chr(27) + "[2J")
    print ret
