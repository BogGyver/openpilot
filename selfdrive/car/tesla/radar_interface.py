#!/usr/bin/env python3
from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.tesla.values import DBC, CAR, CAN_RADAR
from selfdrive.car.interfaces import RadarInterfaceBase
from selfdrive.car.modules.CFG_module import load_bool_param,load_float_param


RADAR_MSGS_A = list(range(0x310, 0x36E, 3))
RADAR_MSGS_B = list(range(0x311, 0x36F, 3))
NUM_POINTS = len(RADAR_MSGS_A)
BOSCH_MAX_DIST = 250.0  # max distance for radar
OBJECT_MIN_PROBABILITY = 50.0

def get_radar_can_parser(CP):
  # Status messages
  signals = [
    ('RADC_HWFail', 'TeslaRadarSguInfo'),
    ('RADC_SGUFail', 'TeslaRadarSguInfo'),
    ('RADC_SensorDirty', 'TeslaRadarSguInfo'),
  ]

  checks = [
    ('TeslaRadarSguInfo', 10),
  ]

  # Radar tracks. There are also raw point clouds available,
  # we don't use those.
  for i in range(NUM_POINTS):
    msg_id_a = RADAR_MSGS_A[i]
    msg_id_b = RADAR_MSGS_B[i]

    # There is a bunch more info in the messages,
    # but these are the only things actually used in openpilot
    signals.extend([
      ('LongDist', msg_id_a),
      ('LongSpeed', msg_id_a),
      ('LatDist', msg_id_a),
      ('LongAccel', msg_id_a),
      ('Meas', msg_id_a),
      ('Tracked', msg_id_a),
      ('Index', msg_id_a),
      ('ProbExist', msg_id_a),
      ('LatSpeed', msg_id_b),
      ('Index2', msg_id_b),
    ])

    checks.extend([
      (msg_id_a, 8),
      (msg_id_b, 8),
    ])

  return CANParser(DBC[CP.carFingerprint]['radar'], signals, checks, CAN_RADAR[CP.carFingerprint])

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.rcp = None
    self.updated_messages = set()
    self.track_id = 0
    self.trigger_msg = RADAR_MSGS_B[-1]
    self.radar_off_can = CP.radarOffCan
    self.fingerprint = CP.carFingerprint
    if not self.radar_off_can:
      self.rcp = get_radar_can_parser(CP)
    self.behindNoseCone = load_bool_param("TinklaTeslaRadarBehindNosecone",False)
    self.radar_offset = load_float_param("TinklaRadarOffset",0.0)
    self.ignoreRadarSGUError = load_bool_param("TinklaTeslaRadarIgnoreSGUError",False)
    self.radarUpsideDown = load_bool_param("TinklaUseTeslaRadarUpsideDown",False)


  def update(self, can_strings):
    if self.rcp is None or self.radar_off_can:
      return super().update(None)

    values = self.rcp.update_strings(can_strings)
    self.updated_messages.update(values)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = car.RadarData.new_message()

    # Errors
    errors = []
    sgu_info = self.rcp.vl['TeslaRadarSguInfo']
    if not self.rcp.can_valid:
      errors.append('canError')
    #ignore RADC_SGUFail on PREAP MODEL S due to retrofit
    if (
          sgu_info['RADC_HWFail'] 
          or 
          (
            sgu_info['RADC_SGUFail'] 
            and 
            self.fingerprint != CAR.PREAP_MODELS
            and 
            not (self.ignoreRadarSGUError)
          ) 
          or 
          (
            sgu_info['RADC_SensorDirty']
            and 
            (
              self.fingerprint != CAR.PREAP_MODELS
              or
              not self.behindNoseCone
            )
          )
    ):
      errors.append('fault')
    ret.errors = errors

    # Radar tracks
    for i in range(NUM_POINTS):
      msg_a = self.rcp.vl[RADAR_MSGS_A[i]]
      msg_b = self.rcp.vl[RADAR_MSGS_B[i]]

      # Make sure msg A and B are together
      if msg_a['Index'] != msg_b['Index2']:
        continue

      # Check if it's a valid track
      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        continue

      #BB Check if it's a valid point
      if ((msg_a["LongDist"] > BOSCH_MAX_DIST) or
          (msg_a["LongDist"] <= 0) or
          (msg_a["ProbExist"] < OBJECT_MIN_PROBABILITY)):
        if i in self.pts:
          del self.pts[i]
        continue

      # New track!
      if i not in self.pts:
        self.pts[i] = car.RadarData.RadarPoint.new_message()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      # Parse track data
      self.pts[i].dRel = msg_a['LongDist']
      self.pts[i].yRel = msg_a['LatDist'] + self.radar_offset
      self.pts[i].vRel = msg_a['LongSpeed']
      self.pts[i].aRel = msg_a['LongAccel']
      self.pts[i].yvRel = msg_b['LatSpeed']
      if self.radarUpsideDown:
        self.pts[i].yRel = - self.pts[i].yRel
        self.pts[i].yvRel = - self.pts[i].yvRel
      self.pts[i].measured = bool(msg_a['Meas'])

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret