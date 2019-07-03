#!/usr/bin/env python
import zmq
import numpy as np
import numpy.matlib
import importlib
from collections import defaultdict, deque

import selfdrive.messaging as messaging
from selfdrive.services import service_list
from selfdrive.controls.lib.latcontrol_helpers import calc_lookahead_offset
from selfdrive.controls.lib.model_parser import ModelParser
from selfdrive.controls.lib.radar_helpers import Track, Cluster, \
                                                 RDR_TO_LDR, NO_FUSION_SCORE

from selfdrive.controls.lib.cluster.fastcluster_py import cluster_points_centroid
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.swaglog import cloudlog
from cereal import car,ui
from common.params import Params
from common.realtime import set_realtime_priority, Ratekeeper
from common.kalman.ekf import EKF, SimpleSensor
from selfdrive.car.tesla.readconfig import read_config_file,CarSettings

DEBUG = False

#vision point
DIMSV = 2
XV, SPEEDV = 0, 1
VISION_POINT = -1


class EKFV1D(EKF):
  def __init__(self):
    super(EKFV1D, self).__init__(False)
    self.identity = numpy.matlib.identity(DIMSV)
    self.state = np.matlib.zeros((DIMSV, 1))
    self.var_init = 1e2   # ~ model variance when probability is 70%, so good starting point
    self.covar = self.identity * self.var_init

    self.process_noise = np.matlib.diag([0.5, 1])

  def calc_transfer_fun(self, dt):
    tf = np.matlib.identity(DIMSV)
    tf[XV, SPEEDV] = dt
    tfj = tf
    return tf, tfj


## fuses camera and radar data for best lead detection
def radard_thread(gctx=None):
  set_realtime_priority(2)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  use_tesla_radar = CarSettings().get_value("useTeslaRadar")
  mocked = (CP.carName == "mock") or ((CP.carName == "tesla") and not use_tesla_radar)
  VM = VehicleModel(CP)
  cloudlog.info("radard got CarParams")
  
  # import the radar from the fingerprint
  cloudlog.info("radard is importing %s", CP.carName)
  RadarInterface = importlib.import_module('selfdrive.car.%s.radar_interface' % CP.carName).RadarInterface
  #we're tesla only branch, alway import the tesla radar
  #RadarInterface = importlib.import_module('selfdrive.car.tesla.radar_interface').RadarInterface
  context = zmq.Context()

  # *** subscribe to features and model from visiond
  poller = zmq.Poller()
  model = messaging.sub_sock(context, service_list['model'].port, conflate=True, poller=poller)
  controls_state_sock = messaging.sub_sock(context, service_list['controlsState'].port, conflate=True, poller=poller)
  live_parameters_sock = messaging.sub_sock(context, service_list['liveParameters'].port, conflate=True, poller=poller)

  # Default parameters
  live_parameters = messaging.new_message()
  live_parameters.init('liveParameters')
  live_parameters.liveParameters.valid = True
  live_parameters.liveParameters.steerRatio = CP.steerRatio
  live_parameters.liveParameters.stiffnessFactor = 1.0

  MP = ModelParser()
  RI = RadarInterface()

  last_md_ts = 0
  last_controls_state_ts = 0

  # *** publish radarState and liveTracks
  radarState = messaging.pub_sock(context, service_list['radarState'].port)
  liveTracks = messaging.pub_sock(context, service_list['liveTracks'].port)
  icCarLR = None
  if (RI.TRACK_RIGHT_LANE or RI.TRACK_LEFT_LANE) and use_tesla_radar:
    icCarLR = messaging.pub_sock(context, service_list['uiIcCarLR'].port)
  path_x = np.arange(0.0, 160.0, 0.1)    # 140 meters is max

  # Time-alignment
  rate = 20.   # tesla radar is at about 16Hz
  tsv = 1./rate
  v_len = 20         # how many speed data points to remember for t alignment with rdr data

  active = 0
  steer_angle = 0.
  steer_override = False

  tracks = defaultdict(dict)

  # Kalman filter stuff:
  ekfv = EKFV1D()
  speedSensorV = SimpleSensor(XV, 1, 2)

  # v_ego
  v_ego = None
  v_ego_hist_t = deque(maxlen=v_len)
  v_ego_hist_v = deque(maxlen=v_len)
  v_ego_t_aligned = 0.

  rk = Ratekeeper(rate, print_delay_threshold=None)
  while 1:
    rr = RI.update()

    ar_pts = {}
    for pt in rr.points:
      ar_pts[pt.trackId] = [pt.dRel + RDR_TO_LDR, pt.yRel, pt.vRel, pt.measured, pt.aRel, pt.yvRel, pt.objectClass, pt.length, pt.trackId+2, pt.movingState]

    # receive the controlsStates
    controls_state = None
    md = None

    for socket, event in poller.poll(0):
      if socket is controls_state_sock:
        controls_state = messaging.recv_one(socket)
      elif socket is model:
        md = messaging.recv_one(socket)
      elif socket is live_parameters_sock:
        live_parameters = messaging.recv_one(socket)
        VM.update_params(live_parameters.liveParameters.stiffnessFactor, live_parameters.liveParameters.steerRatio)

    if controls_state is not None:
      active = controls_state.controlsState.active
      v_ego = controls_state.controlsState.vEgo
      steer_angle = controls_state.controlsState.angleSteers
      steer_override = controls_state.controlsState.steerOverride

      v_ego_hist_v.append(v_ego)
      v_ego_hist_t.append(float(rk.frame)/rate)

      last_controls_state_ts = controls_state.logMonoTime

    if v_ego is None:
      continue

    if md is not None:
      last_md_ts = md.logMonoTime
    

    # *** get path prediction from the model ***
    MP.update(v_ego, md)

    # run kalman filter only if prob is high enough
    if MP.lead_prob > 0.7:
      reading = speedSensorV.read(MP.lead_dist, covar=np.matrix(MP.lead_var))
      ekfv.update_scalar(reading)
      ekfv.predict(tsv)

      # When changing lanes the distance to the lead car can suddenly change,
      # which makes the Kalman filter output large relative acceleration
      if mocked and abs(MP.lead_dist - ekfv.state[XV]) > 2.0:
        ekfv.state[XV] = MP.lead_dist
        ekfv.covar = (np.diag([MP.lead_var, ekfv.var_init]))
        ekfv.state[SPEEDV] = 0.

      ar_pts[VISION_POINT] = (float(ekfv.state[XV]), np.polyval(MP.d_poly, float(ekfv.state[XV])),
                              float(ekfv.state[SPEEDV]), False, 0.,0.,1,0.,1,0)
    else:
      ekfv.state[XV] = MP.lead_dist
      ekfv.covar = (np.diag([MP.lead_var, ekfv.var_init]))
      ekfv.state[SPEEDV] = 0.

      if VISION_POINT in ar_pts:
        del ar_pts[VISION_POINT]

    if (active and not steer_override) or mocked:
      # use path from model (always when mocking as steering is too noisy)
      path_y = np.polyval(MP.d_poly, path_x)
    else:
      # use path from steer, set angle_offset to 0 it does not only report the physical offset
      path_y = calc_lookahead_offset(v_ego, steer_angle, path_x, VM, angle_offset=live_parameters.liveParameters.angleOffsetAverage)[0]

    #BB always use path
    path_y = np.polyval(MP.d_poly, path_x)

    # *** remove missing points from meta data *** 
    for ids in tracks.keys():
      if ids not in ar_pts:
        tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      # ignore standalone vision point, unless we are mocking the radar
      if ids == VISION_POINT and not mocked:
        continue
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      cur_time = float(rk.frame)/rate
      v_ego_t_aligned = np.interp(cur_time - RI.delay, v_ego_hist_t, v_ego_hist_v)

      d_path = np.sqrt(np.amin((path_x - rpt[0]) ** 2 + (path_y - rpt[1]) ** 2))
      # add sign
      d_path *= np.sign(rpt[1] - np.interp(rpt[0], path_x, path_y))

      # create the track if it doesn't exist or it's a new track
      if ids not in tracks:
        tracks[ids] = Track()
      tracks[ids].update(rpt[0], rpt[1], rpt[2], rpt[3], rpt[4],rpt[5],rpt[6],rpt[7],rpt[8],rpt[9],d_path, v_ego_t_aligned, steer_override,use_tesla_radar)

    # allow the vision model to remove the stationary flag if distance and rel speed roughly match
    if VISION_POINT in ar_pts:
      fused_id = None
      best_score = NO_FUSION_SCORE
      for ids in tracks:
        dist_to_vision = np.sqrt((0.5*(ar_pts[VISION_POINT][0] - tracks[ids].dRel)) ** 2 + (2*(ar_pts[VISION_POINT][1] - tracks[ids].yRel)) ** 2)
        rel_speed_diff = abs(ar_pts[VISION_POINT][2] - tracks[ids].vRel)
        tracks[ids].update_vision_score(dist_to_vision, rel_speed_diff)
        if best_score > tracks[ids].vision_score:
          fused_id = ids
          best_score = tracks[ids].vision_score

      if fused_id is not None:
        tracks[fused_id].vision_cnt += 1
        tracks[fused_id].update_vision_fusion()
      #BB renove vision now
      #if (VISION_POINT in ar_pts) and use_tesla_radar:
      #  del ar_pts[VISION_POINT]
        

    if DEBUG:
      print("NEW CYCLE")
      if VISION_POINT in ar_pts:
        print("vision", ar_pts[VISION_POINT])

    idens = list(tracks.keys())
    track_pts = np.array([tracks[iden].get_key_for_cluster() for iden in idens])

    # If we have multiple points, cluster them
    if len(track_pts) > 1:
      cluster_idxs = cluster_points_centroid(track_pts, 2.5)
      clusters = [None] * (max(cluster_idxs) + 1)

      for idx in xrange(len(track_pts)):
        if idx > VISION_POINT:
          cluster_i = cluster_idxs[idx]
          if clusters[cluster_i] is None:
            clusters[cluster_i] = Cluster()
          clusters[cluster_i].add(tracks[idens[idx]])

    elif len(track_pts) == 1:
      # TODO: why do we need this?
      clusters = [Cluster()]
      clusters[0].add(tracks[idens[0]])
    else:
      clusters = []

    if DEBUG:
      for i in clusters:
        print(i)
    # *** extract the lead car ***
    lead_clusters = [c for c in clusters
                     if c.is_potential_lead(v_ego)]
    lead_clusters.sort(key=lambda x: x.dRel)
    lead_len = len(lead_clusters)
    lead1_truck = (len([c for c in lead_clusters
                      if c.is_truck(lead_clusters)]) > 0)

    # *** extract the second lead from the whole set of leads ***
    lead2_clusters = [c for c in lead_clusters
                      if c.is_potential_lead2(lead_clusters)]
    lead2_clusters.sort(key=lambda x: x.dRel)
    lead2_len = len(lead2_clusters)
    lead2_truck = (len([c for c in lead_clusters
                      if c.is_truck(lead2_clusters)]) > 0)

    #################################################################
    #BB For Tesla integration we will also track Left and Right lanes
    #################################################################
    if (RI.TRACK_RIGHT_LANE or RI.TRACK_LEFT_LANE) and use_tesla_radar:
      datrl = ui.ICCarsLR.new_message()
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
      lane_offset = 0. #MP.lane_width
    #LEFT LANE
    if RI.TRACK_LEFT_LANE and use_tesla_radar:
      ll_track_pts = np.array([tracks[iden].get_key_for_cluster_dy(-MP.lane_width) for iden in idens])
      # If we have multiple points, cluster them
      if len(ll_track_pts) > 1:
        ll_cluster_idxs = cluster_points_centroid(ll_track_pts, 2.5)
        ll_clusters = [None] * (max(ll_cluster_idxs) + 1)

        for idx in xrange(len(ll_track_pts)):
          ll_cluster_i = ll_cluster_idxs[idx]

          if ll_clusters[ll_cluster_i] == None:
            ll_clusters[ll_cluster_i] = Cluster()
          ll_clusters[ll_cluster_i].add(tracks[idens[idx]])
      elif len(ll_track_pts) == 1:
        # TODO: why do we need this?
        ll_clusters = [Cluster()]
        ll_clusters[0].add(tracks[idens[0]])
      else:
        ll_clusters = []
      if DEBUG:
        for i in ll_clusters:
          print(i)
      # *** extract the lead car ***
      ll_lead_clusters = [c for c in ll_clusters
                      if c.is_potential_lead_dy(v_ego,-MP.lane_width)]
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
    if RI.TRACK_RIGHT_LANE and use_tesla_radar:
      rl_track_pts = np.array([tracks[iden].get_key_for_cluster_dy(MP.lane_width) for iden in idens])
      # If we have multiple points, cluster them
      if len(rl_track_pts) > 1:
        rl_cluster_idxs = cluster_points_centroid(rl_track_pts, 2.5)
        rl_clusters = [None] * (max(rl_cluster_idxs) + 1)

        for idx in xrange(len(rl_track_pts)):
          rl_cluster_i = rl_cluster_idxs[idx]

          if rl_clusters[rl_cluster_i] == None:
            rl_clusters[rl_cluster_i] = Cluster()
          rl_clusters[rl_cluster_i].add(tracks[idens[idx]])
      elif len(rl_track_pts) == 1:
        # TODO: why do we need this?
        rl_clusters = [Cluster()]
        rl_clusters[0].add(tracks[idens[0]])
      else:
        rl_clusters = []
      if DEBUG:
        for i in rl_clusters:
          print(i)
      # *** extract the lead car ***
      rl_lead_clusters = [c for c in rl_clusters
                      if c.is_potential_lead_dy(v_ego,MP.lane_width)]
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
    if (RI.TRACK_RIGHT_LANE or RI.TRACK_LEFT_LANE) and use_tesla_radar:
      icCarLR.send(datrl.to_bytes())      
    # *** publish radarState ***
    dat = messaging.new_message()
    dat.init('radarState')
    dat.radarState.mdMonoTime = last_md_ts
    dat.radarState.canMonoTimes = list(rr.canMonoTimes)
    dat.radarState.radarErrors = list(rr.errors)
    dat.radarState.controlsStateMonoTime = last_controls_state_ts
    if lead_len > 0:
      dat.radarState.leadOne = lead_clusters[0].toRadarState()
      #print "lead dRel = ", dat.radarState.leadOne.dRel
      if lead1_truck:
        dat.radarState.leadOne.oClass = 0
      if lead2_len > 0:
        dat.radarState.leadTwo = lead2_clusters[0].toRadarState()
        if lead2_truck:
          dat.radarState.leadTwo.oClass = 0
      else:
        dat.radarState.leadTwo.status = False
    else:
      dat.radarState.leadOne.status = False

    dat.radarState.cumLagMs = -rk.remaining*1000.
    radarState.send(dat.to_bytes())

    # *** publish tracks for UI debugging (keep last) ***
    dat = messaging.new_message()
    dat.init('liveTracks', len(tracks))

    for cnt, ids in enumerate(tracks.keys()):
      if DEBUG:
        print("id: %4.0f x:  %4.1f  y: %4.1f  vr: %4.1f d: %4.1f  va: %4.1f  vl: %4.1f  vlk: %4.1f alk: %4.1f  s: %1.0f  v: %1.0f" % \
          (ids, tracks[ids].dRel, tracks[ids].yRel, tracks[ids].vRel,
           tracks[ids].dPath, tracks[ids].vLat,
           tracks[ids].vLead, tracks[ids].vLeadK,
           tracks[ids].aLeadK,
           tracks[ids].stationary,
           tracks[ids].measured))
      dat.liveTracks[cnt] = {
        "trackId": ids,
        "dRel": float(tracks[ids].dRel),
        "yRel": float(tracks[ids].yRel),
        "vRel": float(tracks[ids].vRel),
        "aRel": float(tracks[ids].aRel),
        "stationary": bool(tracks[ids].stationary),
        "oncoming": bool(tracks[ids].oncoming),
      }
    liveTracks.send(dat.to_bytes())

    rk.monitor_time()

def main(gctx=None):
  radard_thread(gctx)

if __name__ == "__main__":
  main()
