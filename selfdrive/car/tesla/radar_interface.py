#!/usr/bin/env python
from cereal import car,tesla
import time
import os
from selfdrive.can.parser import CANParser
from common.realtime import sec_since_boot
from selfdrive.services import service_list
import selfdrive.messaging as messaging
from selfdrive.car.tesla.readconfig import CarSettings

#RADAR_A_MSGS = list(range(0x371, 0x37F , 3))
#RADAR_B_MSGS = list(range(0x372, 0x37F, 3))
BOSCH_MAX_DIST = 150. #max distance for radar
RADAR_A_MSGS = list(range(0x310, 0x36F , 3))
RADAR_B_MSGS = list(range(0x311, 0x36F, 3))
OBJECT_MIN_PROBABILITY = 20.
CLASS_MIN_PROBABILITY = 20.


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

  return CANParser(os.path.splitext(dbc_f)[0], signals, checks, 1)


class RadarInterface(object):
  def __init__(self,CP):
    # radar
    self.pts = {}
    self.extPts = {}
    self.delay = 0.1
    self.useTeslaRadar = CarSettings().get_value("useTeslaRadar")
    self.TRACK_LEFT_LANE = True
    self.TRACK_RIGHT_LANE = True
    self.updated_messages = set()
    if self.useTeslaRadar:
      self.pts = {}
      self.extPts = {}
      self.valid_cnt = {key: 0 for key in RADAR_A_MSGS}
      self.delay = 0.1  # Delay of radar
      self.rcp = _create_radard_can_parser()
      self.logcan = messaging.sub_sock(service_list['can'].port)
      self.radarOffset = CarSettings().get_value("radarOffset")
      self.trackId = 1
      self.trigger_msg = RADAR_B_MSGS[-1]



  def update(self, can_strings):
    # in Bosch radar and we are only steering for now, so sleep 0.05s to keep
    # radard at 20Hz and return no points
    if not self.useTeslaRadar:
      time.sleep(0.05)
      return car.RadarData.new_message(),self.extPts.values()

    tm = int(sec_since_boot() * 1e9)
    if can_strings != None:
      vls = self.rcp.update_strings(tm, can_strings)
      self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None,None

    rr,rrext = self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr,rrext


  def _update(self, updated_messages):
    ret = car.RadarData.new_message()

    for message in updated_messages:
      if not(message in RADAR_A_MSGS):
        if message in self.pts:
          del self.pts[message]
          del self.extPts[message]
        continue
      cpt = self.rcp.vl[message]
      if (cpt['LongDist'] >= BOSCH_MAX_DIST) or (cpt['LongDist']==0) or (not cpt['Tracked']):
        self.valid_cnt[message] = 0    # reset counter
        if message in self.pts:
          del self.pts[message]
          del self.extPts[message]
      elif cpt['Valid'] and (cpt['LongDist'] < BOSCH_MAX_DIST) and (cpt['LongDist'] > 0) and (cpt['ProbExist'] >= OBJECT_MIN_PROBABILITY):
        self.valid_cnt[message] += 1
      else:
        self.valid_cnt[message] = max(self.valid_cnt[message] -1, 0)
        if (self.valid_cnt[message]==0) and (message in self.pts):
          del self.pts[message]
          del self.extPts[message]

      #score = self.rcp.vl[ii+16]['SCORE']
      #print ii, self.valid_cnt[ii], cpt['Valid'], cpt['LongDist'], cpt['LatDist']

      # radar point only valid if it's a valid measurement and score is above 50
      # bosch radar data needs to match Index and Index2 for validity
      # also for now ignore construction elements
      if (cpt['Valid'] or cpt['Tracked'])and (cpt['LongDist']>0) and (cpt['LongDist'] < BOSCH_MAX_DIST) and \
          (cpt['Index'] == self.rcp.vl[message+1]['Index2']) and (self.valid_cnt[message] > 5) and \
          (cpt['ProbExist'] >= OBJECT_MIN_PROBABILITY): # and (self.rcp.vl[ii+1]['Class'] < 4): # and ((self.rcp.vl[ii+1]['MovingState']<3) or (self.rcp.vl[ii+1]['Class'] > 0)):
        if message not in self.pts and ( cpt['Tracked']):
          self.pts[message] = car.RadarData.RadarPoint.new_message()
          self.pts[message].trackId = self.trackId 
          self.extPts[message] = tesla.TeslaRadarPoint.new_message()
          self.extPts[message].trackId = self.trackId 
          self.trackId = (self.trackId + 1) & 0xFFFFFFFFFFFFFFFF
          if self.trackId ==0:
            self.trackId = 1
        if message in self.pts:
          self.pts[message].dRel = cpt['LongDist']  # from front of car
          self.pts[message].yRel = cpt['LatDist']  - self.radarOffset # in car frame's y axis, left is positive
          self.pts[message].vRel = cpt['LongSpeed']
          self.pts[message].aRel = cpt['LongAccel']
          self.pts[message].yvRel = self.rcp.vl[message+1]['LatSpeed']
          self.pts[message].measured = bool(cpt['Meas'])
          self.extPts[message].dz = self.rcp.vl[message+1]['dZ']
          self.extPts[message].movingState = self.rcp.vl[message+1]['MovingState']
          self.extPts[message].length = self.rcp.vl[message+1]['Length']
          self.extPts[message].obstacleProb = cpt['ProbObstacle']
          if self.rcp.vl[message+1]['Class'] >= CLASS_MIN_PROBABILITY:
            self.extPts[message].objectClass = self.rcp.vl[message+1]['Class']
            # for now we will use class 0- unknown stuff to show trucks
            # we will base that on being a class 1 and length of 2 (hoping they meant width not length, but as germans could not decide)
            # 0-unknown 1-four wheel vehicle 2-two wheel vehicle 3-pedestrian 4-construction element
            # going to 0-unknown 1-truck 2-car 3/4-motorcycle/bicycle 5 pedestrian - we have two bits so
            if self.extPts[message].objectClass == 0:
              self.extPts[message].objectClass = 1
            if (self.extPts[message].objectClass == 1) and ((self.extPts[message].length >= 1.8) or (1.6 < self.extPts[message].dz < 4.5)):
              self.extPts[message].objectClass = 0
            if self.extPts[message].objectClass == 4:
              self.extPts[message].objectClass = 1
          else:
            self.extPts[message].objectClass = 1

    ret.points = self.pts.values()
    errors = []
    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors
    return ret,self.extPts.values()

# radar_interface standalone tester
if __name__ == "__main__":
  CP = car.CarParams()
  RI = RadarInterface(CP)
  while 1:
    ret,retext = RI.update(can_strings = None)
    print(chr(27) + "[2J")
    print(ret)
