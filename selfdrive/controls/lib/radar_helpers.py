from common.realtime import DT_MDL
from common.kalman.simple_kalman import KF1D
from selfdrive.config import RADAR_TO_CENTER
from common.numpy_fast import clip, interp

# the longer lead decels, the more likely it will keep decelerating
# TODO is this a good default?
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1   # Kalman filter states enum

# stationary qualification parameters
v_ego_stationary = 4.   # no stationary object flag below this speed

# Lead Kalman Filter params
_VLEAD_A = [[1.0, DT_MDL], [0.0, 1.0]]
_VLEAD_C = [1.0, 0.0]
#_VLEAD_Q = np.matrix([[10., 0.0], [0.0, 100.]])
#_VLEAD_R = 1e3
#_VLEAD_K = np.matrix([[ 0.05705578], [ 0.03073241]])
_VLEAD_K = [[0.1988689], [0.28555364]]


class Track():
  def __init__(self):
    self.ekf = None
    self.cnt = 0
    self.aLeadTau = _LEAD_ACCEL_TAU

  def update(self, d_rel, y_rel, v_rel,measured, a_rel, vy_rel, oClass, length, track_id,movingState, d_path, v_ego_t_aligned,use_tesla_radar):
    
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.aRel = a_rel   # rel acceleration
    self.vLat = vy_rel  # rel lateral speed
    self.oClass = oClass # object class
    self.length = length #length
    self.measured = measured   # measured or estimate
    self.track_id = track_id
    self.dPath = d_path
    self.stationary = (movingState == 3)

    # computed velocity and accelerations
    self.vLead = self.vRel + v_ego_t_aligned

      
    if self.cnt == 0:
      self.kf = KF1D([[self.vLead], [0.0]], _VLEAD_A, _VLEAD_C, _VLEAD_K)
    else:
      self.kf.update(self.vLead)

    self.cnt += 1

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau *= 0.9

  def get_key_for_cluster(self):
    # Weigh y higher since radar is inaccurate in this dimension
    return [self.dRel, self.yRel*2, self.vRel]

  def get_key_for_cluster_dy(self, dy):
    # Weigh y higher since radar is inaccurate in this dimension
    return [self.dRel, (self.yRel-dy)*2, self.vRel]

  def reset_a_lead(self, aLeadK, aLeadTau):
    self.kf = KF1D([[self.vLead], [aLeadK]], _VLEAD_A, _VLEAD_C, _VLEAD_K)
    self.aLeadK = aLeadK
    self.aLeadTau = aLeadTau

def mean(l):
  return sum(l) / len(l)


class Cluster():
  def __init__(self,use_tesla_radar):
    self.tracks = set()
    #BB frame delay for dRel calculation, in seconds
    self.frame_delay = 0.2
    self.useTeslaRadar = use_tesla_radar

  def add(self, t):
    # add the first track
    self.tracks.add(t)

  # TODO: make generic
  @property
  def dRel(self):
    return min([t.dRel for t in self.tracks])

  @property
  def yRel(self):
    return mean([t.yRel for t in self.tracks])

  @property
  def vRel(self):
    return mean([t.vRel for t in self.tracks])

  @property
  def aRel(self):
    return mean([t.aRel for t in self.tracks])

  @property
  def vLead(self):
    return mean([t.vLead for t in self.tracks])

  @property
  def dPath(self):
    return mean([t.dPath for t in self.tracks])

  @property
  def vLat(self):
    return mean([t.vLat for t in self.tracks])

  @property
  def vLeadK(self):
    return mean([t.vLeadK for t in self.tracks])

  @property
  def aLeadK(self):
    if all(t.cnt <= 1 for t in self.tracks):
      return 0.
    else:
      return mean([t.aLeadK for t in self.tracks if t.cnt > 1])

  @property
  def aLeadTau(self):
    if all(t.cnt <= 1 for t in self.tracks):
      return _LEAD_ACCEL_TAU
    else:
      return mean([t.aLeadTau for t in self.tracks if t.cnt > 1])

  @property
  def measured(self):
    return any(t.measured for t in self.tracks)

  @property
  def oClass(self):
    return max([t.oClass for t in self.tracks])

  @property
  def length(self):
    return max([t.length for t in self.tracks])
  
  @property
  def track_id(self):
    return mean([t.track_id for t in self.tracks])
 
  @property
  def stationary(self):
    return all([t.stationary for t in self.tracks])

  def get_RadarState(self, model_prob=0.0):
    dRel_delta_estimate = 0.
    if self.useTeslaRadar:
      dRel_delta_estimate = (self.vRel + self.aRel * self.frame_delay / 2.) * self.frame_delay
    return {
      "dRel": float(self.dRel + dRel_delta_estimate),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "aLeadTau": float(self.aLeadTau),
      "modelProb": model_prob,
      "radar": True,
    }, {
      "trackId": int(self.track_id % 32),
      "oClass": int(self.oClass),
      "length": float(self.length),
    }

  def get_RadarState_from_vision(self, lead_msg, v_ego):
    return {
      "dRel": float(lead_msg.dist - RADAR_TO_CENTER),
      "yRel": float(lead_msg.relY),
      "vRel": float(lead_msg.relVel),
      "vLead": float(v_ego + lead_msg.relVel),
      "vLeadK": float(v_ego + lead_msg.relVel),
      "aLeadK": float(0),
      "aLeadTau": _LEAD_ACCEL_TAU,
      "fcw": False,
      "modelProb": float(lead_msg.prob),
      "radar": False,
      "status": True
    }

  def __str__(self):
    ret = "x: %4.1f  y: %4.1f  v: %4.1f  a: %4.1f" % (self.dRel, self.yRel, self.vRel, self.aLeadK)
    return ret

  def is_potential_lead(self, v_ego):
    # predict cut-ins by extrapolating lateral speed by a lookahead time
    # lookahead time depends on cut-in distance. more attentive for close cut-ins
    # also, above 50 meters the predicted path isn't very reliable

    # the distance at which v_lat matters is higher at higher speed
    lookahead_dist = 40. + v_ego/1.2   #40m at 0mph, ~70m at 80mph

    t_lookahead_v  = [1., 0.]
    t_lookahead_bp = [10., lookahead_dist]

    # average dist
    d_path = self.dPath

    # lat_corr used to be gated on enabled, now always running
    t_lookahead = interp(self.dRel, t_lookahead_bp, t_lookahead_v)

    # correct d_path for lookahead time, considering only cut-ins and no more than 1m impact.
    lat_corr = 0. # BB disables for now : clip(t_lookahead * self.vLat, -1., 1.) if self.measured else 0.

    # consider only cut-ins
    d_path = clip(d_path + lat_corr, min(0., d_path), max(0.,d_path))

    return abs(d_path) < 1.5 and not self.stationary and not self.oncoming

  def is_potential_lead_dy(self, v_ego,dy):
    # predict cut-ins by extrapolating lateral speed by a lookahead time
    # lookahead time depends on cut-in distance. more attentive for close cut-ins
    # also, above 50 meters the predicted path isn't very reliable

    # the distance at which v_lat matters is higher at higher speed
    lookahead_dist = 40. + v_ego/1.2   #40m at 0mph, ~70m at 80mph

    t_lookahead_v  = [1., 0.]
    t_lookahead_bp = [10., lookahead_dist]

    # average dist
    d_path = self.dPath - dy

    # lat_corr used to be gated on enabled, now always running
    t_lookahead = interp(self.dRel, t_lookahead_bp, t_lookahead_v)

    # correct d_path for lookahead time, considering only cut-ins and no more than 1m impact.
    lat_corr = clip(t_lookahead * self.vLat, -1., 1.) if self.measured else 0.

    # consider only cut-ins
    d_path = clip(d_path + lat_corr, min(0., d_path), max(0.,d_path))

    return abs(d_path) < abs(dy/2.)  and not self.stationary #and not self.oncoming

  def is_truck(self,lead_clusters):
    return False
    if len(lead_clusters) > 0:
      lead_cluster = lead_clusters[0]
      # check if the new lead is too close and roughly at the same speed of the first lead:
      # it might just be the second axle of the same vehicle
      return (self.dRel - lead_cluster.dRel < 4.5) and (self.dRel - lead_cluster.dRel > 0.5) and (abs(self.yRel - lead_cluster.yRel) < 2.) and (abs(self.vRel - lead_cluster.vRel) < 0.2)
    else:
      return False

  def is_potential_lead2(self, lead_clusters):
    if len(lead_clusters) > 0:
      lead_cluster = lead_clusters[0]
      return ((self.dRel - lead_cluster.dRel > 8.) and (lead_cluster.oClass > 0))  or ((self.dRel - lead_cluster.dRel > 15.) and (lead_cluster.oClass == 0)) or abs(self.vRel - lead_cluster.vRel) > 1.
    else:
      return False

      
  def potential_low_speed_lead(self, v_ego):
    # stop for stuff in front of you and low speed, even without model confirmation
    return abs(self.yRel) < 1.5 and (v_ego < v_ego_stationary) and self.dRel < 25

  def is_potential_fcw(self, model_prob):
    return model_prob > .9
