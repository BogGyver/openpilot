import os
import math
import numpy as np

# from common.numpy_fast import clip
from common.realtime import sec_since_boot
from selfdrive.services import service_list
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT
from selfdrive.controls.lib.lane_planner import LanePlanner
import selfdrive.messaging as messaging
from selfdrive.car.tesla.readconfig import CarSettings
from cereal import tesla

LOG_MPC = os.environ.get('LOG_MPC', False)


def calc_states_after_delay(states, v_ego, steer_angle, curvature_factor, steer_ratio, delay, long_camera_offset):
  states[0].x = max(v_ego * delay + long_camera_offset, 0.0)
  states[0].psi = v_ego * curvature_factor * math.radians(steer_angle) / steer_ratio * delay
  return states


class PathPlanner(object):
  def __init__(self, CP):
    self.LP = LanePlanner()

    self.last_cloudlog_t = 0

    self.plan = messaging.pub_sock(service_list['pathPlan'].port)
    self.livempc = messaging.pub_sock(service_list['liveMpc'].port)
    self.alca = messaging.pub_sock(service_list['alcaState'].port)

    self.setup_mpc(CP.steerRateCost)
    self.solution_invalid_cnt = 0
    self.path_offset_i = 0.0

  def setup_mpc(self, steer_rate_cost):
    self.libmpc = libmpc_py.libmpc
    self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, steer_rate_cost)

    self.mpc_solution = libmpc_py.ffi.new("log_t *")
    self.cur_state = libmpc_py.ffi.new("state_t *")
    self.cur_state[0].x = 0.0
    self.cur_state[0].y = 0.0
    self.cur_state[0].psi = 0.0
    self.cur_state[0].delta = 0.0

    self.angle_steers_des = 0.0
    self.angle_steers_des_mpc = 0.0
    self.angle_steers_des_prev = 0.0
    self.angle_steers_des_time = 0.0

  def update(self, sm, CP, VM):
    v_ego = sm['carState'].vEgo
    angle_steers = sm['carState'].steeringAngle
    #angle_steers_des = sm['controlsState'].angleSteersDes
    active = sm['controlsState'].active

    angle_offset_average = sm['liveParameters'].angleOffsetAverage
    angle_offset = sm['liveParameters'].angleOffset

    self.LP.update(v_ego, sm['model'],True)

    # Run MPC
    self.angle_steers_des_prev = self.angle_steers_des_mpc
    VM.update_params(sm['liveParameters'].stiffnessFactor, sm['liveParameters'].steerRatio)
    curvature_factor = VM.curvature_factor(v_ego)

    # TODO: Check for active, override, and saturation
    # if active:
    #   self.path_offset_i += self.LP.d_poly[3] / (60.0 * 20.0)
    #   self.path_offset_i = clip(self.path_offset_i, -0.5,  0.5)
    #   self.LP.d_poly[3] += self.path_offset_i
    # else:
    #   self.path_offset_i = 0.0

    # account for actuation delay
    eonToFront = CarSettings().get_value("eonToFront")

    self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset, curvature_factor, VM.sR, CP.steerActuatorDelay, eonToFront)

    v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed
    self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                        list(self.LP.l_poly), list(self.LP.r_poly), list(self.LP.d_poly),
                        self.LP.l_prob, self.LP.r_prob, curvature_factor, v_ego_mpc, self.LP.lane_width)

    # reset to current steer angle if not active or overriding
    if active:
      delta_desired = self.mpc_solution[0].delta[1]
      rate_desired = math.degrees(self.mpc_solution[0].rate[0] * VM.sR)
    else:
      delta_desired = math.radians(angle_steers - angle_offset) / VM.sR
      rate_desired = 0.0

    self.cur_state[0].delta = delta_desired

    self.angle_steers_des_mpc = float(math.degrees(delta_desired * VM.sR) + angle_offset_average)

    #  Check for infeasable MPC solution
    mpc_nans = np.any(np.isnan(list(self.mpc_solution[0].delta)))
    t = sec_since_boot()
    if mpc_nans:
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, CP.steerRateCost)
      self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / VM.sR

      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.mpc_solution[0].cost > 20000. or mpc_nans:   # TODO: find a better way to detect when MPC did not converge
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0
    plan_solution_valid = self.solution_invalid_cnt < 2

    plan_send = messaging.new_message()
    plan_send.init('pathPlan')
    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'liveParameters', 'model'])
    plan_send.pathPlan.laneWidth = float(self.LP.lane_width)
    plan_send.pathPlan.dPoly = [float(x) for x in self.LP.d_poly]
    plan_send.pathPlan.lPoly = [float(x) for x in self.LP.l_poly]
    plan_send.pathPlan.lProb = float(self.LP.l_prob)
    plan_send.pathPlan.rPoly = [float(x) for x in self.LP.r_poly]
    plan_send.pathPlan.rProb = float(self.LP.r_prob)

    plan_send.pathPlan.angleSteers = float(self.angle_steers_des_mpc)
    plan_send.pathPlan.rateSteers = float(rate_desired)
    plan_send.pathPlan.angleOffset = float(self.path_offset_i)
    plan_send.pathPlan.mpcSolutionValid = bool(plan_solution_valid)
    plan_send.pathPlan.paramsValid = bool(sm['liveParameters'].valid)
    plan_send.pathPlan.sensorValid = bool(sm['liveParameters'].sensorValid)
    plan_send.pathPlan.posenetValid = bool(sm['liveParameters'].posenetValid)

    self.plan.send(plan_send.to_bytes())

    alca_state = tesla.ALCAState.new_message()
    #ALCA params
    alca_state.alcaDirection = int(self.LP.ALCAMP.ALCA_direction)
    alca_state.alcaError = bool(self.LP.ALCAMP.ALCA_error)
    alca_state.alcaCancelling = bool(self.LP.ALCAMP.ALCA_cancelling)
    alca_state.alcaEnabled = bool(self.LP.ALCAMP.ALCA_enabled)
    alca_state.alcaLaneWidth = float(self.LP.ALCAMP.ALCA_lane_width)
    alca_state.alcaStep = int(self.LP.ALCAMP.ALCA_step)
    alca_state.alcaTotalSteps = int(self.LP.ALCAMP.ALCA_total_steps)

    self.alca.send(alca_state.to_bytes())

    if LOG_MPC:
      dat = messaging.new_message()
      dat.init('liveMpc')
      dat.liveMpc.x = list(self.mpc_solution[0].x)
      dat.liveMpc.y = list(self.mpc_solution[0].y)
      dat.liveMpc.psi = list(self.mpc_solution[0].psi)
      dat.liveMpc.delta = list(self.mpc_solution[0].delta)
      dat.liveMpc.cost = self.mpc_solution[0].cost
      self.livempc.send(dat.to_bytes())
