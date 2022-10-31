#!/usr/bin/env python3
import math
import numpy as np
from common.numpy_fast import interp

import cereal.messaging as messaging
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from selfdrive.swaglog import cloudlog
from selfdrive.car.tesla.interface import get_tesla_accel_limits
from selfdrive.car.modules.CFG_module import load_bool_param,load_float_param
from selfdrive.controls.lib.vision_turn_controller import VisionTurnController


LON_MPC_STEP = 0.2  # first step is 0.2s
AWARENESS_DECEL = -0.2  # car smoothly decel at .2m/s^2 when user is distracted
#THESE ARE NOT USED HERE, NEW FUNCTION IN TESLA INTERFACE
#A_CRUISE_MIN = -1.2 
#A_CRUISE_MAX_VALS = [1.2, 1.2, 0.8, 0.6]
#A_CRUISE_MAX_BP = [0., 7.5, 15., 25., 40.]

# Lookup table for turns
_A_TOTAL_MAX_V = [2.2, 4.15]
_A_TOTAL_MAX_BP = [20., 40.]

ACCEL_MIN_TURN_SLOWDOWN = - 1.0 # m/s^2

def get_max_accel(CP,v_ego):
  return get_tesla_accel_limits(CP,v_ego)  

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class Planner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()

    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

    #used for slow down in turns
    self.leadsData = None
    self.path_x = np.arange(192)
    self.enable_turn_slowdown = load_bool_param("TinklaTurnSlowdown", True)
    self.turn_slowdown_factor = load_float_param("TinklaTurnSlowdownFactor",0.95)
    self.cruise_source = 'cruise'
    self.vision_turn_controller = VisionTurnController(CP)

  def get_path_length_idx(self, y, distance):
    i = 0
    for val in y:
        if val < distance:
            i = i + 1
    return i

  def update(self, sm):
    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo

    v_cruise_kph = sm['controlsState'].vCruise
    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_state = sm['controlsState'].longControlState
    force_slow_decel = sm['controlsState'].forceDecel


    prev_accel_constraint = True
    disabled = long_control_state == LongCtrlState.off or sm['carState'].gasPressed
    if disabled:
      self.v_desired_filter.x = v_ego
      self.a_desired = a_ego
      # Smoothly changing between accel trajectory is only relevant when OP is driving
      prev_accel_constraint = False

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # Get acceleration and active solutions for custom long mpc.
    self.cruise_source, a_min_sol, v_cruise_sol = self.cruise_solutions(not disabled, self.v_desired_filter.x, self.a_desired,
                                                                        v_cruise, sm)

    accel_limits = get_max_accel(self.CP,v_ego)

    accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    if force_slow_decel:
      # if required so, force a smooth deceleration
      accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
      accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05, a_min_sol)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['carState'], sm['radarState'], v_cruise_sol, prev_accel_constraint=prev_accel_constraint)
    self.v_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 5
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    pm.send('longitudinalPlan', plan_send)


  def cruise_solutions(self, enabled, v_ego, a_ego, v_cruise, sm):

    # Pick solution with lowest velocity target.
    a_solutions = {'cruise': float("inf")}
    v_solutions = {'cruise': v_cruise}

    # Update controllers
    if self.enable_turn_slowdown:
      self.vision_turn_controller.update(enabled, v_ego, a_ego, v_cruise, sm, self.enable_turn_slowdown, self.turn_slowdown_factor)
      #self.speed_limit_controller.update(enabled, v_ego, a_ego, sm, v_cruise, self.events)
      #self.turn_speed_controller.update(enabled, v_ego, a_ego, sm)

      if self.vision_turn_controller.is_active:
        a_solutions['turn'] = self.vision_turn_controller.a_target
        v_solutions['turn'] = self.vision_turn_controller.v_turn

      #if self.speed_limit_controller.is_active:
      #  a_solutions['limit'] = self.speed_limit_controller.a_target
      #  if Params().get_bool("SpeedLimitPercOffset"):
      #    v_solutions['limit'] = self.speed_limit_controller.speed_limit_offseted
      #  else:
      #    v_solutions['limit'] = self.speed_limit_controller.speed_limit + float(int(Params().get("SpeedLimitValueOffset")) * (CV.MPH_TO_MS if not Params().get_bool("IsMetric") else CV.KPH_TO_MS))

      #if self.turn_speed_controller.is_active:
      #  a_solutions['turnlimit'] = self.turn_speed_controller.a_target
      #  v_solutions['turnlimit'] = self.turn_speed_controller.speed_limit

    source = min(v_solutions, key=v_solutions.get)

    return source, a_solutions[source], v_solutions[source]