#!/usr/bin/env python
from cereal import car
import time
import os
import zmq
from selfdrive.can.parser import CANParser
from common.realtime import sec_since_boot
from selfdrive.services import service_list
import selfdrive.messaging as messaging
from selfdrive.car.honda.readconfig import read_config_file,CarSettings

#RADAR_A_MSGS = list(range(0x371, 0x37F , 3))
#RADAR_B_MSGS = list(range(0x372, 0x37F, 3))
BOSCH_MAX_DIST = 150. #max distance for radar
RADAR_A_MSGS = list(range(0x310, 0x36F , 3))
RADAR_B_MSGS = list(range(0x311, 0x36F, 3))
OBJECT_MIN_PROBABILITY = 60.
CLASS_MIN_PROBABILITY = 50.


# Tesla Bosch firmware has 32 objects in all objects or a selected set of the 5 we should look at
# definetly switch to all objects when calibrating but most likely use select set of 5 for normal use
USE_ALL_OBJECTS = True

def _create_radard_can_parser():
  dbc_f = 'teslaradar.dbc'

  msg_a_n = len(RADAR_A_MSGS)
  msg_b_n = len(RADAR_B_MSGS)

  signals = zip(['LongDist'] * msg_a_n +  ['LatDist'] * msg_a_n +
                ['LongSpeed'] * msg_a_n + ['LongAccel'] * msg_a_n + 
                ['Valid'] * msg_a_n + ['Tracked'] * msg_a_n + 
                ['Meas'] * msg_a_n + ['ProbExist'] * msg_a_n + 
                ['Index'] * msg_a_n + ['ProbObstacle'] * msg_a_n + 
                ['LatSpeed'] * msg_b_n + ['Index2'] * msg_b_n +
                ['Class'] * msg_b_n + ['ProbClass'] * msg_b_n + 
                ['Length'] * msg_b_n + ['dZ'] * msg_b_n + ['MovingState'] * msg_b_n,
                RADAR_A_MSGS * 10 + RADAR_B_MSGS * 7,
                [255.] * msg_a_n + [0.] * msg_a_n + [0.] * msg_a_n + [0.] * msg_a_n + 
                [0] * msg_a_n + [0] * msg_a_n + [0] * msg_a_n + [0.] * msg_a_n +
                [0] * msg_a_n + [0.] * msg_a_n + [0.] * msg_b_n + [0] * msg_b_n +
                [0] * msg_b_n + [0.] * msg_b_n + [0.] * msg_b_n +[0.] * msg_b_n + [0]* msg_b_n)

  checks = zip(RADAR_A_MSGS + RADAR_B_MSGS, [20]*(msg_a_n + msg_b_n))

  return CANParser(os.path.splitext(dbc_f)[0], signals, checks, 2)

def _create_nidec_can_parser():
  dbc_f = 'acura_ilx_2016_nidec.dbc'
  radar_messages = [0x400] + range(0x430, 0x43A) + range(0x440, 0x446)
  signals = list(zip(['RADAR_STATE'] +
                ['LONG_DIST'] * 16 + ['NEW_TRACK'] * 16 + ['LAT_DIST'] * 16 +
                ['REL_SPEED'] * 16,
                [0x400] + radar_messages[1:] * 4,
                [0] + [255] * 16 + [1] * 16 + [0] * 16 + [0] * 16))
  checks = list(zip([0x445], [20]))

  return CANParser(os.path.splitext(dbc_f)[0], signals, checks, 1)


class RadarInterface(object):
  def __init__(self,CP):
    # radar
    self.pts = {}
    self.delay = 0.1
    self.useTeslaRadar = CarSettings().get_value("useTeslaRadar")
    self.TRACK_LEFT_LANE = False
    self.TRACK_RIGHT_LANE = False
    self.radar_off_can = CP.radarOffCan
    if self.useTeslaRadar:
      self.pts = {}
      self.valid_cnt = {key: 0 for key in RADAR_A_MSGS}
      self.delay = 0.05  # Delay of radar
      self.rcp = _create_radard_can_parser()
      context = zmq.Context()
      self.logcan = messaging.sub_sock(context, service_list['can'].port)
      self.radarOffset = CarSettings().get_value("radarOffset")
    else:
      # Nidec
      self.pts = {}
      self.track_id = 0
      self.radar_fault = False
      self.radar_wrong_config = False
      self.delay = 0.1  # Delay of radar
      self.rcp = _create_nidec_can_parser()
      context = zmq.Context()
      self.logcan = messaging.sub_sock(context, service_list['can'].port)

  def update(self):

    updated_messages = set()
    ret = car.RadarData.new_message()

    # in Bosch radar and we are only steering for now, so sleep 0.05s to keep
    # radard at 20Hz and return no points
    if self.radar_off_can:
      time.sleep(0.05)
      return ret

    while 1:
      tm = int(sec_since_boot() * 1e9)
      _, vls = self.rcp.update(tm, True)
      updated_messages.update(vls)
      if 0x445 in updated_messages:
        break

    for ii in updated_messages:
      cpt = self.rcp.vl[ii]
      if ii == 0x400:
        # check for radar faults
        self.radar_fault = cpt['RADAR_STATE'] != 0x79
        self.radar_wrong_config = cpt['RADAR_STATE'] == 0x69
      elif cpt['LONG_DIST'] < 255:
        if ii not in self.pts or cpt['NEW_TRACK']:
          self.pts[ii] = car.RadarData.RadarPoint.new_message()
          self.pts[ii].trackId = self.track_id
          self.track_id += 1
        self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
        self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
        self.pts[ii].vRel = cpt['REL_SPEED']
        self.pts[ii].aRel = float('nan')
        self.pts[ii].yvRel = float('nan')
        self.pts[ii].measured = True
      else:
        if ii in self.pts:
          del self.pts[ii]

    errors = []
    if not self.rcp.can_valid:
      errors.append("commIssue")
    if self.radar_fault:
      errors.append("fault")
    if self.radar_wrong_config:
      errors.append("wrongConfig")
    ret.errors = errors
    ret.canMonoTimes = canMonoTimes

    ret.points = self.pts.values()

    return ret


if __name__ == "__main__":
  class CarParams:
    radarOffCan = False

  CP = CarParams()
  RI = RadarInterface(CP)
  while 1:
    ret = RI.update()
    print(chr(27) + "[2J")
    print(ret)
