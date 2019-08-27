#!/usr/bin/env python
import numpy as np
import numpy.matlib
import importlib
import zmq
from collections import defaultdict, deque

import selfdrive.messaging as messaging
from selfdrive.services import service_list
from selfdrive.controls.lib.radar_helpers import Track, Cluster
from selfdrive.config import RADAR_TO_CENTER
from selfdrive.controls.lib.cluster.fastcluster_py import cluster_points_centroid
from selfdrive.swaglog import cloudlog
from cereal import car,log,tesla
from common.params import Params
from common.realtime import set_realtime_priority, Ratekeeper, DT_MDL
from selfdrive.car.tesla.readconfig import read_config_file,CarSettings

DEBUG = False

#vision point
DIMSV = 2
XV, SPEEDV = 0, 1
VISION_POINT = -1
RDR_TO_LDR = 0.

# Time-alignment
rate = 1. / DT_MDL  # model and radar are both at 20Hz
v_len = 20   # how many speed data points to remember for t alignment with rdr data


def laplacian_cdf(x, mu, b):
  b = np.max([b, 1e-4])
  return np.exp(-abs(x-mu)/b)


def match_vision_to_cluster(v_ego, lead, clusters):
  # match vision point to best statistical cluster match
  probs = []
  offset_vision_dist = lead.dist - RADAR_TO_CENTER
  for c in clusters:
    prob_d = laplacian_cdf(c.dRel, offset_vision_dist, lead.std)
    prob_y = laplacian_cdf(c.yRel, lead.relY, lead.relYStd)
    prob_v = laplacian_cdf(c.vRel, lead.relVel, lead.relVelStd)
    # This is isn't exactly right, but good heuristic
    combined_prob = prob_d * prob_y * prob_v
    probs.append(combined_prob)
  idx = np.argmax(probs)
  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  dist_sane = abs(clusters[idx].dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(clusters[idx].vRel - lead.relVel) < 10) or (v_ego + clusters[idx].vRel > 2)
  if dist_sane and vel_sane:
    return idx
  else:
    return None

def get_rrext_by_trackId(rrext,trackId):
  if rrext is not None:
    for p in rrext:
      if p.trackId == trackId:
        return p
  return None

def get_lead(v_ego, ready, clusters, lead_msg, low_speed_override=True):
  # Determine leads, this is where the essential logic happens
  if len(clusters) > 0 and ready and lead_msg.prob > .5:
    lead_idx = match_vision_to_cluster(v_ego, lead_msg, clusters)
  else:
    lead_idx = None

  lead_dict = {'status': False}
  lead_dict_ext = {'trackId': 1, 'oClass': 0, 'length': 0.}
  if lead_idx is not None:
    lead_dict,lead_dict_ext = clusters[lead_idx].get_RadarState(lead_msg.prob)
  elif (lead_idx is None) and ready and (lead_msg.prob > .5):
    lead_dict = Cluster().get_RadarState_from_vision(lead_msg, v_ego)

  if low_speed_override:
    low_speed_clusters = [c for c in clusters if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_clusters) > 0:
      lead_idx = np.argmin([c.dRel for c in low_speed_clusters])
      if (not lead_dict['status']) or (low_speed_clusters[lead_idx].dRel < lead_dict['dRel']):
        lead_dict,lead_dict_ext = low_speed_clusters[lead_idx].get_RadarState()

  return lead_dict,lead_dict_ext


class RadarD(object):
  def __init__(self, mocked, RI):
    self.current_time = 0
    self.mocked = mocked
    self.RI = RI
    self.tracks = defaultdict(dict)

    self.last_md_ts = 0
    self.last_controls_state_ts = 0

    self.active = 0

    # v_ego
    self.v_ego = 0.
    self.v_ego_hist_t = deque([0], maxlen=v_len)
    self.v_ego_hist_v = deque([0], maxlen=v_len)
    self.v_ego_t_aligned = 0.
    self.ready = False
    self.icCarLR = None
    if (RI.TRACK_RIGHT_LANE or RI.TRACK_LEFT_LANE) and CarSettings().get_value("useTeslaRadar"):
      self.icCarLR = messaging.pub_sock(service_list['uiIcCarLR'].port)
    
    self.lane_width = 3.0
    #only used for left and right lanes
    self.path_x = np.arange(0.0, 160.0, 0.1)    # 160 meters is max
    self.poller = zmq.Poller()
    self.pathPlanSocket = messaging.sub_sock(service_list['pathPlan'].port, conflate=True, poller=self.poller)
    self.dPoly = [0.,0.,0.,0.]

  def update(self, frame, delay, sm, rr, has_radar,rrext):
    self.current_time = 1e-9*max([sm.logMonoTime[key] for key in sm.logMonoTime.keys()])
    use_tesla_radar = CarSettings().get_value("useTeslaRadar")
    if sm.updated['controlsState']:
      self.active = sm['controlsState'].active
      self.v_ego = sm['controlsState'].vEgo
      self.v_ego_hist_v.append(self.v_ego)
      self.v_ego_hist_t.append(float(frame)/rate)
    if sm.updated['model']:
      self.ready = True

    for socket, _ in self.poller.poll(0):
      if socket is self.pathPlanSocket:
        pp = messaging.recv_one(self.pathPlanSocket).pathPlan
        self.lane_width = pp.laneWidth
        self.dPoly = pp.dPoly

    path_y = np.polyval(self.dPoly, self.path_x)

    ar_pts = {}
    for pt in rr.points:
      extpt = get_rrext_by_trackId(rrext,pt.trackId)
      ar_pts[pt.trackId] = [pt.dRel + RDR_TO_LDR, pt.yRel, pt.vRel, pt.measured, pt.aRel, pt.yvRel, extpt.objectClass, extpt.length, pt.trackId+2, extpt.movingState]
    # *** remove missing points from meta data ***
    for ids in self.tracks.keys():
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      cur_time = float(frame)/rate
      self.v_ego_t_aligned = np.interp(cur_time - delay, self.v_ego_hist_t, self.v_ego_hist_v)

      # distance relative to path
      d_path = np.sqrt(np.amin((self.path_x - rpt[0]) ** 2 + (path_y - rpt[1]) ** 2))
      # add sign
      d_path *= np.sign(rpt[1] - np.interp(rpt[0], self.path_x, path_y))

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track()
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], rpt[3], rpt[4],rpt[5],rpt[6],rpt[7],rpt[8],rpt[9], d_path, self.v_ego_t_aligned,use_tesla_radar)

    idens = list(self.tracks.keys())
    track_pts = np.array([self.tracks[iden].get_key_for_cluster() for iden in idens])


    # If we have multiple points, cluster them
    if len(track_pts) > 1:
      cluster_idxs = cluster_points_centroid(track_pts, 2.5)
      clusters = [None] * (max(cluster_idxs) + 1)

      for idx in xrange(len(track_pts)):
        cluster_i = cluster_idxs[idx]
        if clusters[cluster_i] is None:
          clusters[cluster_i] = Cluster()
        clusters[cluster_i].add(self.tracks[idens[idx]])
    elif len(track_pts) == 1:
      # FIXME: cluster_point_centroid hangs forever if len(track_pts) == 1
      cluster_idxs = [0]
      clusters = [Cluster()]
      clusters[0].add(self.tracks[idens[0]])
    else:
      clusters = []

    # if a new point, reset accel to the rest of the cluster
    for idx in xrange(len(track_pts)):
      if self.tracks[idens[idx]].cnt <= 1:
        aLeadK = clusters[cluster_idxs[idx]].aLeadK
        aLeadTau = clusters[cluster_idxs[idx]].aLeadTau
        self.tracks[idens[idx]].reset_a_lead(aLeadK, aLeadTau)
    
    ### START REVIEW SECTION

    #################################################################
    #BB For Tesla integration we will also track Left and Right lanes
    #################################################################
    if (self.RI.TRACK_RIGHT_LANE or self.RI.TRACK_LEFT_LANE) and use_tesla_radar:
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
    if self.RI.TRACK_LEFT_LANE and use_tesla_radar:
      ll_track_pts = np.array([self.tracks[iden].get_key_for_cluster_dy(-self.lane_width) for iden in idens])
      # If we have multiple points, cluster them
      if len(ll_track_pts) > 1:
        ll_cluster_idxs = cluster_points_centroid(ll_track_pts, 2.5)
        ll_clusters = [None] * (max(ll_cluster_idxs) + 1)

        for idx in xrange(len(ll_track_pts)):
          ll_cluster_i = ll_cluster_idxs[idx]

          if ll_clusters[ll_cluster_i] == None:
            ll_clusters[ll_cluster_i] = Cluster()
          ll_clusters[ll_cluster_i].add(self.tracks[idens[idx]])
      elif len(ll_track_pts) == 1:
        # TODO: why do we need this?
        ll_clusters = [Cluster()]
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
    if self.RI.TRACK_RIGHT_LANE and use_tesla_radar:
      rl_track_pts = np.array([self.tracks[iden].get_key_for_cluster_dy(self.lane_width) for iden in idens])
      # If we have multiple points, cluster them
      if len(rl_track_pts) > 1:
        rl_cluster_idxs = cluster_points_centroid(rl_track_pts, 2.5)
        rl_clusters = [None] * (max(rl_cluster_idxs) + 1)

        for idx in xrange(len(rl_track_pts)):
          rl_cluster_i = rl_cluster_idxs[idx]

          if rl_clusters[rl_cluster_i] == None:
            rl_clusters[rl_cluster_i] = Cluster()
          rl_clusters[rl_cluster_i].add(self.tracks[idens[idx]])
      elif len(rl_track_pts) == 1:
        # TODO: why do we need this?
        rl_clusters = [Cluster()]
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
    if (self.RI.TRACK_RIGHT_LANE or self.RI.TRACK_LEFT_LANE) and use_tesla_radar:
      self.icCarLR.send(datrl.to_bytes())      

    ### END REVIEW SECTION
    

    # *** publish radarState ***
    dat = messaging.new_message()
    dat.init('radarState')
    dat.valid = sm.all_alive_and_valid(service_list=['controlsState', 'model'])
    dat.radarState.mdMonoTime = self.last_md_ts
    dat.radarState.canMonoTimes = list(rr.canMonoTimes)
    dat.radarState.radarErrors = list(rr.errors)
    dat.radarState.controlsStateMonoTime = self.last_controls_state_ts

    datext = tesla.ICLeads.new_message()
    l1x = tesla.TeslaLeadPoint.new_message()
    l2x = tesla.TeslaLeadPoint.new_message()
    if has_radar:
      l1d,l1x = get_lead(self.v_ego, self.ready, clusters, sm['model'].lead, low_speed_override=True)
      l2d,l2x = get_lead(self.v_ego, self.ready, clusters, sm['model'].leadFuture, low_speed_override=False)
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
def radard_thread(gctx=None):
  set_realtime_priority(2)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  use_tesla_radar = CarSettings().get_value("useTeslaRadar")
  mocked = (CP.carName == "mock") or ((CP.carName == "tesla") and not use_tesla_radar)
  cloudlog.info("radard got CarParams")

  # import the radar from the fingerprint
  cloudlog.info("radard is importing %s", CP.carName)
  RadarInterface = importlib.import_module('selfdrive.car.%s.radar_interface' % CP.carName).RadarInterface

  can_sock = messaging.sub_sock(service_list['can'].port)
  sm = messaging.SubMaster(['model', 'controlsState', 'liveParameters'])

  RI = RadarInterface(CP)

  # *** publish radarState and liveTracks
  radarState = messaging.pub_sock(service_list['radarState'].port)
  liveTracks = messaging.pub_sock(service_list['liveTracks'].port)
  icLeads = messaging.pub_sock(service_list['uiIcLeads'].port)

  rk = Ratekeeper(rate, print_delay_threshold=None)
  RD = RadarD(mocked, RI)

  has_radar = not CP.radarOffCan or mocked
  last_md_ts = 0.
  v_ego = 0.

  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    rr,rrext = RI.update(can_strings)

    if rr is None:
      continue

    sm.update(0)

    if sm.updated['controlsState']:
      v_ego = sm['controlsState'].vEgo

    

    dat,datext = RD.update(rk.frame, RI.delay, sm, rr, has_radar, rrext)
    dat.radarState.cumLagMs = -rk.remaining*1000.

    radarState.send(dat.to_bytes())
    icLeads.send(datext.to_bytes())

    # *** publish tracks for UI debugging (keep last) ***
    tracks = RD.tracks
    dat = messaging.new_message()
    dat.init('liveTracks', len(tracks))

    for cnt, ids in enumerate(tracks.keys()):
      dat.liveTracks[cnt] = {
        "trackId": ids,
        "dRel": float(tracks[ids].dRel),
        "yRel": float(tracks[ids].yRel),
        "vRel": float(tracks[ids].vRel),
      }
    liveTracks.send(dat.to_bytes())

    rk.monitor_time()


def main(gctx=None):
  radard_thread(gctx)


if __name__ == "__main__":
  main()
