#!/usr/bin/env python3
import importlib
import math
from collections import defaultdict, deque
import numpy as np

import cereal.messaging as messaging
from cereal import car,tesla
from common.numpy_fast import interp
from common.params import Params
from common.realtime import Ratekeeper, set_realtime_priority
from selfdrive.config import RADAR_TO_CAMERA
from selfdrive.controls.lib.cluster.fastcluster_py import cluster_points_centroid
from selfdrive.controls.lib.radar_helpers import Cluster, Track
from selfdrive.swaglog import cloudlog
from selfdrive.car.tesla.readconfig import CarSettings

DEBUG = False
RDR_TO_LDR = 0

class KalmanParams():
  def __init__(self, dt):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.1s and 1.0s
    assert dt > .01 and dt < .1, "Radar time step must be between .01s and 0.1s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [dt * 0.01 for dt in range(1, 11)]
    K0 = [0.12288, 0.14557, 0.16523, 0.18282, 0.19887, 0.21372, 0.22761, 0.24069, 0.2531, 0.26491]
    K1 = [0.29666, 0.29331, 0.29043, 0.28787, 0.28555, 0.28342, 0.28144, 0.27958, 0.27783, 0.27617]
    self.K = [[interp(dt, dts, K0)], [interp(dt, dts, K1)]]


def laplacian_cdf(x, mu, b):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_cluster(v_ego, lead, clusters):
  # match vision point to best statistical cluster match
  offset_vision_dist = lead.dist - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_cdf(c.dRel, offset_vision_dist, lead.std)
    prob_y = laplacian_cdf(c.yRel, lead.relY, lead.relYStd)
    prob_v = laplacian_cdf(c.vRel, lead.relVel, lead.relVelStd)

    # This is isn't exactly right, but good heuristic
    return prob_d * prob_y * prob_v

  cluster = max(clusters, key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(cluster.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(cluster.vRel - lead.relVel) < 10) or (v_ego + cluster.vRel > 2)
  if dist_sane and vel_sane:
    return cluster
  else:
    return None

def get_rrext_by_trackId(rrext,trackId):
  if rrext is not None:
    for p in rrext:
      if p.trackId == trackId:
        return p
  return None

def get_lead(v_ego, ready, clusters, lead_msg, low_speed_override=True,use_tesla_radar=False):
  # Determine leads, this is where the essential logic happens
  if len(clusters) > 0 and ready and lead_msg.prob > .5:
    cluster = match_vision_to_cluster(v_ego, lead_msg, clusters)
  else:
    cluster = None

  lead_dict = {'status': False}
  lead_dict_ext = {'trackId': 1, 'oClass': 1, 'length': 0.}
  # temporary for development purposes: we set the default lead vehicle type to truck (=0) to distinguish between vision (truck) and radar leads (car) in IC
  if use_tesla_radar:
    lead_dict_ext['oClass'] = 0
  if cluster is not None:
    lead_dict,lead_dict_ext = cluster.get_RadarState(lead_msg.prob)
  elif (cluster is None) and ready and (lead_msg.prob > .5):
    lead_dict = Cluster(use_tesla_radar).get_RadarState_from_vision(lead_msg, v_ego)

  if low_speed_override:
    low_speed_clusters = [c for c in clusters if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_clusters) > 0:
      closest_cluster = min(low_speed_clusters, key=lambda c: c.dRel)

      # Only choose new cluster if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_cluster.dRel < lead_dict['dRel']):
        lead_dict,lead_dict_ext = closest_cluster.get_RadarState()

  return lead_dict,lead_dict_ext


class RadarD():
  def __init__(self, radar_ts, RI,use_tesla_radar, delay=0):
    self.current_time = 0
    self.RI = RI
    self.tracks = defaultdict(dict)
    self.kalman_params = KalmanParams(radar_ts)

    self.last_md_ts = 0
    self.last_controls_state_ts = 0

    self.active = 0

    # v_ego
    self.v_ego = 0.
    self.v_ego_hist = deque([0], maxlen=delay+1)

    self.ready = False
    self.icCarLR = None
    self.use_tesla_radar = use_tesla_radar
    if self.use_tesla_radar:
      if (RI.TRACK_RIGHT_LANE or RI.TRACK_LEFT_LANE):
        self.icCarLR = messaging.pub_sock('uiIcCarLR')
    
    self.lane_width = 3.0
    #only used for left and right lanes
    self.path_x = np.arange(0.0, 160.0, 0.1)    # 160 meters is max
    self.dPoly = [0.,0.,0.,0.]

  def update(self, frame, sm, rr, has_radar,rrext):
    self.current_time = 1e-9*max([sm.logMonoTime[key] for key in sm.logMonoTime.keys()])
    
    if sm.updated['controlsState']:
      self.active = sm['controlsState'].active
      self.v_ego = sm['controlsState'].vEgo
      self.v_ego_hist.append(self.v_ego)
    if sm.updated['model']:
      self.ready = True
    if self.use_tesla_radar:
      if sm.updated['pathPlan']:
        self.lane_width = sm['pathPlan'].laneWidth
        self.dPoly = sm['pathPlan'].dPoly

    path_y = np.polyval(self.dPoly, self.path_x)

    ar_pts = {}
    for pt in rr.points:
      if rrext:
        extpt = get_rrext_by_trackId(rrext,pt.trackId)
        ar_pts[pt.trackId] = [pt.dRel, pt.yRel, pt.vRel, pt.measured, pt.aRel, pt.yvRel, extpt.objectClass, extpt.length, pt.trackId+2, extpt.movingState]
      else:
        ar_pts[pt.trackId] = [pt.dRel, pt.yRel, pt.vRel, pt.measured, pt.aRel, pt.yvRel, 1, 0, pt.trackId+2, 1]

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # distance relative to path
      d_path = np.sqrt(np.amin((self.path_x - rpt[0]) ** 2 + (path_y - rpt[1]) ** 2))
      # add sign
      d_path *= np.sign(rpt[1] - np.interp(rpt[0], self.path_x, path_y))

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], rpt[3], rpt[4],rpt[5],rpt[6],rpt[7],rpt[8],rpt[9], d_path, self.v_ego,v_lead,self.use_tesla_radar)

    idens = list(sorted(self.tracks.keys()))
    track_pts = list([self.tracks[iden].get_key_for_cluster() for iden in idens])


    # If we have multiple points, cluster them
    if len(track_pts) > 1:
      cluster_idxs = cluster_points_centroid(track_pts, 2.5)
      clusters = [None] * (max(cluster_idxs) + 1)

      for idx in range(len(track_pts)):
        cluster_i = cluster_idxs[idx]
        if clusters[cluster_i] is None:
          clusters[cluster_i] = Cluster(self.use_tesla_radar)
        clusters[cluster_i].add(self.tracks[idens[idx]])
    elif len(track_pts) == 1:
      # FIXME: cluster_point_centroid hangs forever if len(track_pts) == 1
      cluster_idxs = [0]
      clusters = [Cluster(self.use_tesla_radar)]
      clusters[0].add(self.tracks[idens[0]])
    else:
      clusters = []

    # if a new point, reset accel to the rest of the cluster
    for idx in range(len(track_pts)):
      if self.tracks[idens[idx]].cnt <= 1:
        aLeadK = clusters[cluster_idxs[idx]].aLeadK
        aLeadTau = clusters[cluster_idxs[idx]].aLeadTau
        self.tracks[idens[idx]].reset_a_lead(aLeadK, aLeadTau)
    
    ### START REVIEW SECTION

    #################################################################
    #BB For Tesla integration we will also track Left and Right lanes
    #################################################################
    if self.use_tesla_radar:
      if (self.RI.TRACK_RIGHT_LANE or self.RI.TRACK_LEFT_LANE):
        datrl = tesla.ICCarsLR.new_message()
        datrl.v1Type = int(0)
        datrl.v1Dx = float(0.)
        datrl.v1Vrel = float(0.)
        datrl.v1Dy = float(0.)
        datrl.v1Id = int(0)
        datrl.v2Type = int(0) 
        datrl.v2Dx = float(0.)
        datrl.v2Vrel = float(0.)
        datrl.v2Dy = float(0.)
        datrl.v2Id = int(0)
        datrl.v3Type = int(0)
        datrl.v3Dx = float(0.)
        datrl.v3Vrel = float(0.)
        datrl.v3Dy = float(0.)
        datrl.v3Id = int(0)
        datrl.v4Type = int(0) 
        datrl.v4Dx = float(0.)
        datrl.v4Vrel = float(0.)
        datrl.v4Dy = float(0.)
        datrl.v4Id = int(0)
        lane_offset = 0. 
      #LEFT LANE
      if self.RI.TRACK_LEFT_LANE:
        ll_track_pts = np.array([self.tracks[iden].get_key_for_cluster_dy(-self.lane_width) for iden in idens])
        # If we have multiple points, cluster them
        if len(ll_track_pts) > 1:
          ll_cluster_idxs = cluster_points_centroid(ll_track_pts, 2.5)
          ll_clusters = [None] * (max(ll_cluster_idxs) + 1)

          for idx in range(len(ll_track_pts)):
            ll_cluster_i = ll_cluster_idxs[idx]

            if ll_clusters[ll_cluster_i] == None:
              ll_clusters[ll_cluster_i] = Cluster(self.use_tesla_radar)
            ll_clusters[ll_cluster_i].add(self.tracks[idens[idx]])
        elif len(ll_track_pts) == 1:
          # TODO: why do we need this?
          ll_clusters = [Cluster(self.use_tesla_radar)]
          ll_clusters[0].add(self.tracks[idens[0]])
        else:
          ll_clusters = []
        if DEBUG:
          for i in ll_clusters:
            print(i)
        # *** extract the lead car ***
        ll_lead_clusters = [c for c in ll_clusters
                        if c.is_potential_lead_dy(self.v_ego,-self.lane_width)]
        ll_lead_clusters.sort(key=lambda x: x.dRel)
        ll_lead_len = len(ll_lead_clusters)
        ll_lead1_truck = (len([c for c in ll_lead_clusters
                        if c.is_truck(ll_lead_clusters)]) > 0)

        # *** extract the second lead from the whole set of leads ***
        ll_lead2_clusters = [c for c in ll_lead_clusters
                          if c.is_potential_lead2(ll_lead_clusters)]
        ll_lead2_clusters.sort(key=lambda x: x.dRel)
        ll_lead2_len = len(ll_lead2_clusters)
        ll_lead2_truck = (len([c for c in ll_lead_clusters
                        if c.is_truck(ll_lead2_clusters)]) > 0)
        # publish data
        if ll_lead_len > 0:
          datrl.v1Type = int(ll_lead_clusters[0].oClass)
          if datrl.v1Type == 1 and ll_lead1_truck:
              datrl.v1Type = 0
          datrl.v1Dx = float(ll_lead_clusters[0].dRel)
          datrl.v1Vrel = float(ll_lead_clusters[0].vRel)
          datrl.v1Dy = float(-ll_lead_clusters[0].yRel - lane_offset)
          datrl.v1Id = int(ll_lead_clusters[0].track_id % 32)
          if ll_lead2_len > 0:
            datrl.v2Type = int(ll_lead2_clusters[0].oClass)
            if datrl.v2Type == 1 and ll_lead2_truck:
              datrl.v2Type = 0
            datrl.v2Dx = float(ll_lead2_clusters[0].dRel)
            datrl.v2Vrel = float(ll_lead2_clusters[0].vRel)
            datrl.v2Dy = float(-ll_lead2_clusters[0].yRel - lane_offset) 
            datrl.v2Id = int(ll_lead2_clusters[0].track_id % 32)
      #RIGHT LANE
      if self.RI.TRACK_RIGHT_LANE:
        rl_track_pts = np.array([self.tracks[iden].get_key_for_cluster_dy(self.lane_width) for iden in idens])
        # If we have multiple points, cluster them
        if len(rl_track_pts) > 1:
          rl_cluster_idxs = cluster_points_centroid(rl_track_pts, 2.5)
          rl_clusters = [None] * (max(rl_cluster_idxs) + 1)

          for idx in range(len(rl_track_pts)):
            rl_cluster_i = rl_cluster_idxs[idx]

            if rl_clusters[rl_cluster_i] == None:
              rl_clusters[rl_cluster_i] = Cluster(self.use_tesla_radar)
            rl_clusters[rl_cluster_i].add(self.tracks[idens[idx]])
        elif len(rl_track_pts) == 1:
          # TODO: why do we need this?
          rl_clusters = [Cluster(self.use_tesla_radar)]
          rl_clusters[0].add(self.tracks[idens[0]])
        else:
          rl_clusters = []
        if DEBUG:
          for i in rl_clusters:
            print(i)
        # *** extract the lead car ***
        rl_lead_clusters = [c for c in rl_clusters
                        if c.is_potential_lead_dy(self.v_ego,self.lane_width)]
        rl_lead_clusters.sort(key=lambda x: x.dRel)
        rl_lead_len = len(rl_lead_clusters)
        rl_lead1_truck = (len([c for c in rl_lead_clusters
                        if c.is_truck(rl_lead_clusters)]) > 0)
        # *** extract the second lead from the whole set of leads ***
        rl_lead2_clusters = [c for c in rl_lead_clusters
                          if c.is_potential_lead2(rl_lead_clusters)]
        rl_lead2_clusters.sort(key=lambda x: x.dRel)
        rl_lead2_len = len(rl_lead2_clusters)
        rl_lead2_truck = (len([c for c in rl_lead_clusters
                        if c.is_truck(rl_lead2_clusters)]) > 0)
        # publish data
        if rl_lead_len > 0:
          datrl.v3Type = int(rl_lead_clusters[0].oClass) 
          if datrl.v3Type == 1 and rl_lead1_truck:
            datrl.v3Type = 0
          datrl.v3Dx = float(rl_lead_clusters[0].dRel)
          datrl.v3Vrel = float(rl_lead_clusters[0].vRel)
          datrl.v3Dy = float(-rl_lead_clusters[0].yRel+ lane_offset)
          datrl.v3Id = int(rl_lead_clusters[0].track_id % 32)
          if rl_lead2_len > 0:
            datrl.v4Type = int(rl_lead2_clusters[0].oClass)
            if datrl.v4Type == 1 and rl_lead2_truck:
              datrl.v4Type = 0
            datrl.v4Dx = float(rl_lead2_clusters[0].dRel)
            datrl.v4Vrel = float(rl_lead2_clusters[0].vRel)
            datrl.v4Dy = float(-rl_lead2_clusters[0].yRel + lane_offset)
            datrl.v4Id = int(rl_lead2_clusters[0].track_id % 32)
      if (self.RI.TRACK_RIGHT_LANE or self.RI.TRACK_LEFT_LANE):
        self.icCarLR.send(datrl.to_bytes())      

      ### END REVIEW SECTION
    

    # *** publish radarState ***
    dat = messaging.new_message('radarState')
    dat.valid = sm.all_alive_and_valid(service_list=['controlsState', 'model'])
    dat.radarState.mdMonoTime = self.last_md_ts
    dat.radarState.canMonoTimes = list(rr.canMonoTimes)
    dat.radarState.radarErrors = list(rr.errors)
    dat.radarState.controlsStateMonoTime = self.last_controls_state_ts

    datext = tesla.ICLeads.new_message()
    if has_radar:
      l1d,l1x = get_lead(self.v_ego, self.ready, clusters, sm['model'].lead, low_speed_override=True,use_tesla_radar=self.use_tesla_radar)
      l2d,l2x = get_lead(self.v_ego, self.ready, clusters, sm['model'].leadFuture, low_speed_override=False, use_tesla_radar=self.use_tesla_radar)
      dat.radarState.leadOne = l1d
      dat.radarState.leadTwo = l2d
    
      datext.lead1trackId = l1x['trackId']
      datext.lead1oClass = l1x['oClass']
      datext.lead1length = l1x['length']
      datext.lead2trackId = l2x['trackId']
      datext.lead2oClass = l2x['oClass']
      datext.lead2length = l2x['length']
    return dat, datext


# fuses camera and radar data for best lead detection
def radard_thread(sm=None, pm=None, can_sock=None):
  set_realtime_priority(2)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  use_tesla_radar = CarSettings().get_value("useTeslaRadar")
  cloudlog.info("radard got CarParams")

  # import the radar from the fingerprint
  cloudlog.info("radard is importing %s", CP.carName)
  RadarInterface = importlib.import_module('selfdrive.car.%s.radar_interface' % CP.carName).RadarInterface

  if can_sock is None:
    can_sock = messaging.sub_sock('can')

  if sm is None:
    if CP.carName == "tesla":
      sm = messaging.SubMaster(['model', 'controlsState', 'liveParameters', 'pathPlan'])
    else:
      sm = messaging.SubMaster(['model', 'controlsState', 'liveParameters'])


  # *** publish radarState and liveTracks
  if pm is None:
    pm = messaging.PubMaster(['radarState', 'liveTracks'])
  if CP.carName == "tesla":
    icLeads = messaging.pub_sock('uiIcLeads')
    ahbInfo = messaging.pub_sock('ahbInfo')

  RI = RadarInterface(CP)

  rk = Ratekeeper(1.0 / CP.radarTimeStep, print_delay_threshold=None)
  RD = RadarD(CP.radarTimeStep, RI, use_tesla_radar,RI.delay)

  has_radar = CP.openpilotLongitudinalControl or not CP.radarOffCan
  v_ego = 0.
  print("Working with ",CP.carName," with radarOffCan=",CP.radarOffCan)
  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)


    if CP.carName == "tesla":
      rr,rrext,ahbCarDetected = RI.update(can_strings,v_ego)
    else:
      rr = RI.update(can_strings)
      rrext = None
      ahbCarDetected = False
    if rr is None:
      continue    

    sm.update(0)

    if sm.updated['controlsState']:
      v_ego = sm['controlsState'].vEgo

    dat,datext = RD.update(rk.frame, sm, rr, has_radar, rrext)
    dat.radarState.cumLagMs = -rk.remaining*1000.

    pm.send('radarState', dat)

    if CP.carName == "tesla":
      icLeads.send(datext.to_bytes())
      ahbInfoMsg = tesla.AHBinfo.new_message()
      ahbInfoMsg.source = 0
      ahbInfoMsg.radarCarDetected = ahbCarDetected
      ahbInfoMsg.cameraCarDetected = False
      ahbInfo.send(ahbInfoMsg.to_bytes())


    # *** publish tracks for UI debugging (keep last) ***
    tracks = RD.tracks
    dat = messaging.new_message('liveTracks', len(tracks))

    for cnt, ids in enumerate(sorted(tracks.keys())):
      dat.liveTracks[cnt] = {
        "trackId": ids,
        "dRel": float(tracks[ids].dRel),
        "yRel": float(tracks[ids].yRel),
        "vRel": float(tracks[ids].vRel),
      }
    pm.send('liveTracks', dat)

    rk.monitor_time()


def main(sm=None, pm=None, can_sock=None):
  radard_thread(sm,pm,can_sock)

if __name__ == "__main__":
    main()
