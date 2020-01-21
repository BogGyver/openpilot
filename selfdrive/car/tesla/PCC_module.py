#!/usr/bin/env python
from selfdrive.car.tesla import teslacan
from selfdrive.car.tesla.speed_utils.fleet_speed import FleetSpeed
from selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from common.numpy_fast import clip, interp
from selfdrive.car.tesla.values import CruiseState, CruiseButtons
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.speed_smoother import speed_smoother
from selfdrive.controls.lib.planner import calc_cruise_accel_limits
import selfdrive.messaging as messaging
import time
import math
from collections import OrderedDict
from common.params import Params
import json

_DT = 0.05    # 10Hz in our case, since we don't want to process more than once the same radarState message
_DT_MPC = _DT

#Reset the PID completely on disengage of PCC
RESET_PID_ON_DISENGAGE = False

# TODO: these should end up in values.py at some point, probably variable by trim
# Accel limits
MAX_RADAR_DISTANCE = 120. #max distance to take in consideration radar reading
MAX_PEDAL_VALUE = 112.
PEDAL_HYST_GAP = 1.0  # don't change pedal command for small oscilalitons within this value
# Cap the pedal to go from 0 to max in 4 seconds
PEDAL_MAX_UP = MAX_PEDAL_VALUE * _DT / 4
# Cap the pedal to go from max to 0 in 0.4 seconds
PEDAL_MAX_DOWN = MAX_PEDAL_VALUE * _DT / 0.4

# min safe distance in meters. Roughly 2 car lengths.
MIN_SAFE_DIST_M = 6.

#BBTODO: move the vehicle variables; maybe make them speed variable
TORQUE_LEVEL_ACC = 0.
TORQUE_LEVEL_DECEL = -30.

MIN_PCC_V_KPH = 0. #
MAX_PCC_V_KPH = 270.

ANGLE_STOP_ACCEL = 10. #this should be speed dependent

MIN_CAN_SPEED = 0.3  #TODO: parametrize this in car interface

# Pull the cruise stalk twice in this many ms for a 'double pull'
STALK_DOUBLE_PULL_MS = 750

V_PID_FILE = '/data/params/pidParams'

class Mode():
  label = None

class OpMode(Mode):
  label = 'OP'

class FollowMode(Mode):
  label = 'FOLLOW'
  
class PCCModes():
  _all_modes = [OpMode(), FollowMode()]
  _mode_map = {mode.label : mode for mode in _all_modes}
  BUTTON_NAME = 'pedal'
  BUTTON_ABREVIATION = 'PCC'
  
  @classmethod
  def from_label(cls, label):
    return cls._mode_map.get(label, OpMode())
    
  @classmethod
  def from_buttons(cls, cstm_btns):
    return cls.from_label(cstm_btns.get_button_label2(cls.BUTTON_NAME))
    
  @classmethod
  def is_selected(cls, mode, cstm_butns):
    """Tell if the UI buttons are set to the given mode"""
    button_mode = cls.from_buttons(cstm_butns)
    if not (isinstance(mode, Mode) and isinstance(button_mode, Mode)):
      return False
    return mode.label == button_mode.label
    
  @classmethod
  def labels(cls):
    return [mode.label for mode in cls._all_modes]
    

def tesla_compute_gb(accel, speed):
  return float(accel)  / 3.

  
def max_v_in_mapped_curve_ms(map_data, pedal_set_speed_kph):
  """Use HD map data to limit speed in sharper turns."""
  if map_data and map_data.curvatureValid:
    pedal_set_speed_ms = pedal_set_speed_kph * CV.KPH_TO_MS
    # Max lateral acceleration, used to caclulate how much to slow down in turns
    a_y_max = 1.85  # m/s^2
    curvature = abs(map_data.curvature)
    v_curvature_ms = math.sqrt(a_y_max / max(1e-4, curvature))
    time_to_turn_s = max(0, map_data.distToTurn / max(pedal_set_speed_ms, 1.))
    v_approaching_turn_ms = OrderedDict([
      # seconds til turn, max allowed velocity
      (0, pedal_set_speed_ms),
      (8, v_curvature_ms)
    ])
    return _interp_map(time_to_turn_s, v_approaching_turn_ms)
  else:
    return None



class PCCState():
  # Possible state of the PCC system, following the DI_cruiseState naming scheme.
  OFF = 0         # Disabled by UI (effectively never happens since button switches over to ACC mode).
  STANDBY = 1     # Ready to be engaged.
  ENABLED = 2     # Engaged.
  NOT_READY = 9   # Not ready to be engaged due to the state of the car.


def _current_time_millis():
  return int(round(time.time() * 1000))


#this is for the pedal cruise control
class PCCController():
  def __init__(self,carcontroller):
    self.CC = carcontroller
    self.human_cruise_action_time = 0
    self.pcc_available = self.prev_pcc_available = False
    self.pedal_timeout_frame = 0
    self.accelerator_pedal_pressed = self.prev_accelerator_pedal_pressed = False
    self.automated_cruise_action_time = 0
    self.last_angle = 0.
    self.radarState = messaging.sub_sock('radarState', conflate=True)
    self.live_map_data = messaging.sub_sock('liveMapData', conflate=True)
    self.lead_1 = None
    self.last_update_time = 0
    self.enable_pedal_cruise = False
    self.stalk_pull_time_ms = 0
    self.prev_stalk_pull_time_ms = -1000
    self.prev_pcm_acc_status = 0
    self.prev_cruise_buttons = CruiseButtons.IDLE
    self.pedal_speed_kph = 0.
    self.speed_limit_kph = 0.
    self.prev_speed_limit_kph = 0.
    self.pedal_idx = 0
    self.pedal_steady = 0.
    self.prev_tesla_accel = 0.
    self.prev_tesla_pedal = 0.
    self.torqueLevel_last = 0.
    self.prev_v_ego = 0.
    self.PedalForZeroTorque = 18. #starting number, works on my S85
    self.lastTorqueForPedalForZeroTorque = TORQUE_LEVEL_DECEL
    self.v_pid = 0.
    self.a_pid = 0.
    self.last_output_gb = 0.
    self.last_speed_kph = None
    #for smoothing the changes in speed
    self.v_acc_start = 0.0
    self.a_acc_start = 0.0
    self.v_acc = 0.0
    self.v_acc_sol = 0.0
    self.v_acc_future = 0.0
    self.a_acc = 0.0
    self.a_acc_sol = 0.0
    self.v_cruise = 0.0
    self.a_cruise = 0.0
    #Long Control
    self.LoC = None
    #when was radar data last updated?
    self.lead_last_seen_time_ms = 0
    self.continuous_lead_sightings = 0
    self.params = Params()
    average_speed_over_x_suggestions = 6 # 0.3 seconds (20x a second)
    self.fleet_speed = FleetSpeed(average_speed_over_x_suggestions)
    
  def load_pid(self):
    try:
      v_pid_json = open(V_PID_FILE)
      data = json.load(v_pid_json)
      if (self.LoC):
        if self.LoC.pid:
          self.LoC.pid.p = data['p']
          self.LoC.pid.i = data['i']
          self.LoC.pid.f = data['f']
      else:
        print("self.LoC not initialized!")
    except :
      print("file not present, creating at next reset")

    #Helper function for saving the PCC pid constants across drives
  def save_pid(self, pid):
    data = {}
    data['p'] = pid.p
    data['i'] = pid.i
    data['f'] = pid.f
    try:
      with open(V_PID_FILE , 'w') as outfile :
        json.dump(data, outfile)
    except IOError:
      print("PDD pid parameters could not be saved to file")

  def reset(self, v_pid):
    if self.LoC and RESET_PID_ON_DISENGAGE:
      self.LoC.reset(v_pid)

  def update_stat(self, CS, frame):
    if not self.LoC:
      self.LoC = LongControl(CS.CP, tesla_compute_gb)
      # Get v_id from the stored file when initiating the LoC and reset_on_disengage==false
      if (not RESET_PID_ON_DISENGAGE): 
        self.load_pid()

    self._update_pedal_state(CS, frame)

    can_sends = []
    if not self.pcc_available:
      timed_out = frame >= self.pedal_timeout_frame
      if timed_out or CS.pedal_interceptor_state > 0:
        if self.prev_pcc_available:
          CS.UE.custom_alert_message(4, "Pedal Interceptor %s" % ("timed out" if timed_out else "fault (state %s)" % CS.pedal_interceptor_state), 200, 4)
        if frame % 50 == 0:
          # send reset command
          idx = self.pedal_idx
          self.pedal_idx = (self.pedal_idx + 1) % 16
          can_sends.append(teslacan.create_pedal_command_msg(0, 0, idx))
      return can_sends

    prev_enable_pedal_cruise = self.enable_pedal_cruise
    # disable on brake
    if CS.brake_pressed and self.enable_pedal_cruise:
      self.enable_pedal_cruise = False
      self.reset(0.)

    # process any stalk movement
    curr_time_ms = _current_time_millis()
    speed_uom_kph = 1.
    if CS.imperial_speed_units:
      speed_uom_kph = CV.MPH_TO_KPH
    if (CS.cruise_buttons == CruiseButtons.MAIN and
        self.prev_cruise_buttons != CruiseButtons.MAIN):
      self.prev_stalk_pull_time_ms = self.stalk_pull_time_ms
      self.stalk_pull_time_ms = curr_time_ms
      double_pull = self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms < STALK_DOUBLE_PULL_MS
      ready = (CS.cstm_btns.get_button_status(PCCModes.BUTTON_NAME) > PCCState.OFF
               and (CruiseState.is_off(CS.pcm_acc_status)) or CS.forcePedalOverCC)
      if ready and double_pull:
        # A double pull enables ACC. updating the max ACC speed if necessary.
        self.enable_pedal_cruise = True
        self.reset(CS.v_ego)
        # Increase PCC speed to match current, if applicable.
        # We round the target speed in the user's units of measurement to avoid jumpy speed readings
        current_speed_kph_uom_rounded = int(CS.v_ego * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
        self.pedal_speed_kph = max(current_speed_kph_uom_rounded, self.speed_limit_kph)
    # Handle pressing the cancel button.
    elif CS.cruise_buttons == CruiseButtons.CANCEL:
      self.enable_pedal_cruise = False
      self.pedal_speed_kph = 0. 
      self.stalk_pull_time_ms = 0
      self.prev_stalk_pull_time_ms = -1000
    # Handle pressing up and down buttons.
    elif (self.enable_pedal_cruise 
          and CS.cruise_buttons != self.prev_cruise_buttons):
      # Real stalk command while PCC is already enabled. Adjust the max PCC speed if necessary.
      # We round the target speed in the user's units of measurement to avoid jumpy speed readings
      actual_speed_kph_uom_rounded = int(CS.v_ego * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
      if CS.cruise_buttons == CruiseButtons.RES_ACCEL:
        self.pedal_speed_kph = max(self.pedal_speed_kph, actual_speed_kph_uom_rounded) + speed_uom_kph
      elif CS.cruise_buttons == CruiseButtons.RES_ACCEL_2ND:
        self.pedal_speed_kph = max(self.pedal_speed_kph, actual_speed_kph_uom_rounded) + 5 * speed_uom_kph
      elif CS.cruise_buttons == CruiseButtons.DECEL_SET:
        self.pedal_speed_kph = self.pedal_speed_kph - speed_uom_kph
      elif CS.cruise_buttons == CruiseButtons.DECEL_2ND:
        self.pedal_speed_kph = self.pedal_speed_kph - 5 * speed_uom_kph
      # Clip PCC speed between 0 and 170 KPH.
      self.pedal_speed_kph = clip(self.pedal_speed_kph, MIN_PCC_V_KPH, MAX_PCC_V_KPH)
    # If something disabled cruise control, disable PCC too
    elif self.enable_pedal_cruise and CS.pcm_acc_status and not CS.forcePedalOverCC:
      self.enable_pedal_cruise = False
    # A single pull disables PCC (falling back to just steering). Wait some time
    # in case a double pull comes along.
    elif (self.enable_pedal_cruise
          and curr_time_ms - self.stalk_pull_time_ms > STALK_DOUBLE_PULL_MS
          and self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms > STALK_DOUBLE_PULL_MS): 
      self.enable_pedal_cruise = False
    
    # Notify if PCC was toggled
    if prev_enable_pedal_cruise and not self.enable_pedal_cruise:
      CS.UE.custom_alert_message(3, "PCC Disabled", 150, 4)
      CS.cstm_btns.set_button_status(PCCModes.BUTTON_NAME, PCCState.STANDBY)
      self.fleet_speed.reset_averager()
      #save the pid parameters to params file
      self.save_pid(self.LoC.pid)
    elif self.enable_pedal_cruise and not prev_enable_pedal_cruise:
      CS.UE.custom_alert_message(2, "PCC Enabled", 150)
      CS.cstm_btns.set_button_status(PCCModes.BUTTON_NAME, PCCState.ENABLED)

    # Update the UI to show whether the current car state allows PCC.
    if CS.cstm_btns.get_button_status(PCCModes.BUTTON_NAME) in [PCCState.STANDBY, PCCState.NOT_READY]:
      if CruiseState.is_off(CS.pcm_acc_status) or CS.forcePedalOverCC:
        CS.cstm_btns.set_button_status(PCCModes.BUTTON_NAME, PCCState.STANDBY)
      else:
        CS.cstm_btns.set_button_status(PCCModes.BUTTON_NAME, PCCState.NOT_READY)
          
    # Update prev state after all other actions.
    self.prev_cruise_buttons = CS.cruise_buttons
    self.prev_pcm_acc_status = CS.pcm_acc_status

    return can_sends
    
  def update_pdl(self, enabled, CS, frame, actuators, pcm_speed, speed_limit_ms, set_speed_limit_active, speed_limit_offset,alca_enabled):
    idx = self.pedal_idx

    self.prev_speed_limit_kph = self.speed_limit_kph

    ######################################################################################
    # Determine pedal "zero"
    #
    #save position for cruising (zero acc, zero brake, no torque) when we are above 10 MPH
    ######################################################################################
    if (CS.torqueLevel < TORQUE_LEVEL_ACC
        and CS.torqueLevel > TORQUE_LEVEL_DECEL
        and CS.v_ego >= 10.* CV.MPH_TO_MS
        and abs(CS.torqueLevel) < abs(self.lastTorqueForPedalForZeroTorque)
        and self.prev_tesla_accel > 0.):
      self.PedalForZeroTorque = self.prev_tesla_accel
      self.lastTorqueForPedalForZeroTorque = CS.torqueLevel
      #print ("Detected new Pedal For Zero Torque at %s" % (self.PedalForZeroTorque))
      #print ("Torque level at detection %s" % (CS.torqueLevel))
      #print ("Speed level at detection %s" % (CS.v_ego * CV.MS_TO_MPH))

    if set_speed_limit_active and speed_limit_ms > 0:
      self.speed_limit_kph = (speed_limit_ms + speed_limit_offset) * CV.MS_TO_KPH
      if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph):
        self.pedal_speed_kph = self.speed_limit_kph
        # reset MovingAverage for fleet speed when speed limit changes
        self.fleet_speed.reset_averager()
    else: # reset internal speed limit, so double pull doesn't set higher speed than current (e.g. after leaving the highway)
      self.speed_limit_kph = 0.
    self.pedal_idx = (self.pedal_idx + 1) % 16

    if not self.pcc_available or not enabled:
      return 0., 0, idx
    # Alternative speed decision logic that uses the lead car's distance
    # and speed more directly.
    # Bring in the lead car distance from the radarState feed
    radSt = messaging.recv_one_or_none(self.radarState)
    mapd = messaging.recv_one_or_none(self.live_map_data)
    if radSt is not None:
      self.lead_1 = radSt.radarState.leadOne
      if _is_present(self.lead_1):
        self.lead_last_seen_time_ms = _current_time_millis()
        self.continuous_lead_sightings += 1
      else:
        self.continuous_lead_sightings = 0
      

    v_ego = CS.v_ego
    following = self.lead_1.status and self.lead_1.dRel < MAX_RADAR_DISTANCE and self.lead_1.vLeadK > v_ego and self.lead_1.aLeadK > 0.0
    accel_limits = [float(x) for x in calc_cruise_accel_limits(v_ego)]
    accel_limits[1] *= _accel_limit_multiplier(CS, self.lead_1)
    accel_limits[0] = _decel_limit(accel_limits[0], CS.v_ego, self.lead_1, CS, self.pedal_speed_kph)
    jerk_limits = [min(-0.1, accel_limits[0]/2.), max(0.1, accel_limits[1]/2.)]  # TODO: make a separate lookup for jerk tuning
    #accel_limits = limit_accel_in_turns(v_ego, CS.angle_steers, accel_limits, CS.CP)

    output_gb = 0
    ####################################################################
    # this mode (Follow) uses the Follow logic created by JJ for ACC
    #
    # once the speed is detected we have to use our own PID to determine
    # how much accel and break we have to do
    ####################################################################
    if PCCModes.is_selected(FollowMode(), CS.cstm_btns):
      enabled = self.enable_pedal_cruise and self.LoC.long_control_state in [LongCtrlState.pid, LongCtrlState.stopping]
      # determine if pedal is pressed by human
      self.prev_accelerator_pedal_pressed = self.accelerator_pedal_pressed
      self.accelerator_pedal_pressed = CS.pedal_interceptor_value > 10
      #reset PID if we just lifted foot of accelerator
      if (not self.accelerator_pedal_pressed) and self.prev_accelerator_pedal_pressed:
        self.reset(CS.v_ego)

      if self.enable_pedal_cruise and not self.accelerator_pedal_pressed:
        self.v_pid = self.calc_follow_speed_ms(CS,alca_enabled)

        if mapd is not None:
          v_curve = max_v_in_mapped_curve_ms(mapd.liveMapData, self.pedal_speed_kph)
          if v_curve:
            self.v_pid = min(self.v_pid, v_curve)
        # take fleet speed into consideration
        self.v_pid = min(self.v_pid, self.fleet_speed.adjust(CS, self.pedal_speed_kph * CV.KPH_TO_MS, frame))
        # cruise speed can't be negative even if user is distracted
        self.v_pid = max(self.v_pid, 0.)

        jerk_min, jerk_max = _jerk_limits(CS.v_ego, self.lead_1, self.v_pid * CV.MS_TO_KPH, self.lead_last_seen_time_ms, CS)
        self.v_cruise, self.a_cruise = speed_smoother(self.v_acc_start, self.a_acc_start,
                                                      self.v_pid,
                                                      accel_limits[1], accel_limits[0],
                                                      jerk_limits[1], jerk_limits[0], #jerk_max, jerk_min,
                                                      _DT_MPC)
        
        # cruise speed can't be negative even is user is distracted
        self.v_cruise = max(self.v_cruise, 0.)

        self.v_acc = self.v_cruise
        self.a_acc = self.a_cruise
        self.v_acc_future = self.v_pid

        # Interpolation of trajectory
        self.a_acc_sol = self.a_acc_start + (_DT / _DT_MPC) * (self.a_acc - self.a_acc_start)
        self.v_acc_sol = self.v_acc_start + _DT * (self.a_acc_sol + self.a_acc_start) / 2.0


        self.v_acc_start = self.v_acc_sol
        self.a_acc_start = self.a_acc_sol

        # we will try to feed forward the pedal position.... we might want to feed the last_output_gb....
        # op feeds forward self.a_acc_sol
        # it's all about testing now.
        vTarget = clip(self.v_acc_sol, 0, self.v_cruise)
        self.vTargetFuture = clip(self.v_acc_future, 0, self.v_pid)
        feedforward = self.a_acc_sol
        #feedforward = self.last_output_gb
        t_go, t_brake = self.LoC.update(self.enable_pedal_cruise, CS.v_ego, CS.brake_pressed != 0, CS.standstill, False, 
                    self.v_cruise , vTarget, self.vTargetFuture, feedforward, CS.CP)
        output_gb = t_go - t_brake
        #print ("Output GB Follow:", output_gb)
      else:
        self.reset(CS.v_ego)
        #print ("PID reset")
        output_gb = 0.
        starting = self.LoC.long_control_state == LongCtrlState.starting
        a_ego = min(CS.a_ego, 0.0)
        reset_speed = MIN_CAN_SPEED if starting else CS.v_ego
        reset_accel = CS.CP.startAccel if starting else a_ego
        self.v_acc = reset_speed
        self.a_acc = reset_accel
        self.v_acc_start = reset_speed
        self.a_acc_start = reset_accel
        self.v_cruise = reset_speed
        self.a_cruise = reset_accel
        self.v_acc_sol = reset_speed
        self.a_acc_sol = reset_accel
        self.v_pid = reset_speed
        self.last_speed_kph = None

    ##############################################################
    # This mode uses the longitudinal MPC built in OP
    #
    # we use the values from actuator.accel and actuator.brake
    ##############################################################
    elif PCCModes.is_selected(OpMode(), CS.cstm_btns):
      output_gb = actuators.gas - actuators.brake
      self.v_pid = -10.

    self.last_output_gb = output_gb
    # accel and brake
    apply_accel = clip(output_gb, 0., _accel_pedal_max(CS.v_ego, self.v_pid, self.lead_1, self.prev_tesla_accel, CS))
    MPC_BRAKE_MULTIPLIER = 6.
    apply_brake = -clip(output_gb * MPC_BRAKE_MULTIPLIER, _brake_pedal_min(CS.v_ego, self.v_pid, self.lead_1, CS, self.pedal_speed_kph), 0.)

    # if speed is over 5mph, the "zero" is at PedalForZeroTorque; otherwise it is zero
    pedal_zero = 0.
    if CS.v_ego >= 5.* CV.MPH_TO_MS:
      pedal_zero = self.PedalForZeroTorque
    tesla_brake = clip((1. - apply_brake) * pedal_zero, 0, pedal_zero)
    tesla_accel = clip(apply_accel * (MAX_PEDAL_VALUE - pedal_zero) , 0, MAX_PEDAL_VALUE - pedal_zero)
    tesla_pedal = tesla_brake + tesla_accel

    tesla_pedal = self.pedal_hysteresis(tesla_pedal, enabled)
    
    tesla_pedal = clip(tesla_pedal, self.prev_tesla_pedal - PEDAL_MAX_DOWN, self.prev_tesla_pedal + PEDAL_MAX_UP)
    tesla_pedal = clip(tesla_pedal, 0., MAX_PEDAL_VALUE) if self.enable_pedal_cruise else 0.
    enable_pedal = 1. if self.enable_pedal_cruise else 0.
    
    self.torqueLevel_last = CS.torqueLevel
    self.prev_tesla_pedal = tesla_pedal * enable_pedal
    self.prev_tesla_accel = apply_accel * enable_pedal
    self.prev_v_ego = CS.v_ego


    return self.prev_tesla_pedal, enable_pedal, idx

  # function to calculate the cruise speed based on a safe follow distance
  def calc_follow_speed_ms(self, CS, alca_enabled):
    # Make sure we were able to populate lead_1.
    if self.lead_1 is None:
      return None, None, None
    # dRel is in meters.
    lead_dist_m = self.lead_1.dRel
    if not CS.useTeslaRadar:
      lead_dist_m = _visual_radar_adjusted_dist_m(lead_dist_m, CS)
    # Grab the relative speed.
    v_rel = self.lead_1.vRel if abs(self.lead_1.vRel) > .5 else 0
    a_rel = self.lead_1.aRel if abs(self.lead_1.aRel) > .5 else 0
    rel_speed_kph = (v_rel + 0 * CS.apFollowTimeInS * a_rel) * CV.MS_TO_KPH
    # v_ego is in m/s, so safe_distance is in meters.
    safe_dist_m = _safe_distance_m(CS.v_ego,CS)
    # Current speed in kph
    actual_speed_kph = CS.v_ego * CV.MS_TO_KPH
    # speed and brake to issue
    if self.last_speed_kph is None:
      self.last_speed_kph = actual_speed_kph
    new_speed_kph = self.last_speed_kph
    ###   Logic to determine best cruise speed ###
    if self.enable_pedal_cruise:
      # If no lead is present, accel up to max speed
      if lead_dist_m == 0 or lead_dist_m > MAX_RADAR_DISTANCE:
        new_speed_kph = self.pedal_speed_kph
      elif lead_dist_m > 0:
        #BB Use the Kalman lead speed and acceleration
        lead_absolute_speed_kph = actual_speed_kph + rel_speed_kph #(self.lead_1.vLeadK + _DT * self.lead_1.aLeadK) * CV.MS_TO_KPH
        rel_speed_kph = lead_absolute_speed_kph - actual_speed_kph
        if lead_dist_m < MIN_SAFE_DIST_M and rel_speed_kph >= 3:
        # If lead is going faster, but we're not at a safe distance, hold 
        # speed and let the lead car move father away from us
          new_speed_kph = actual_speed_kph
        # If lead is not going faster than 3kpm from us, lets slow down a
        # bit to get him moving faster relative to us
        elif lead_dist_m < MIN_SAFE_DIST_M:
          new_speed_kph = MIN_PCC_V_KPH
        # In a 10 meter cruise zone, lets match the car in front 
        elif safe_dist_m + 2 > lead_dist_m > MIN_SAFE_DIST_M: # BB we might want to try this and rel_speed_kph > 0: 
          min_vrel_kph_map = OrderedDict([
            # (distance in m, min allowed relative kph)
            (0.5 * safe_dist_m, 3.0),
            (0.8 * safe_dist_m, 2.0),
            (1.0 * safe_dist_m, 0.0)])
          min_vrel_kph = _interp_map(lead_dist_m, min_vrel_kph_map)
          new_speed_kph = lead_absolute_speed_kph - min_vrel_kph
        else:
          # Force speed into a band that is generally slower than lead if too
          # close, and faster than lead if too far. Allow a range of speeds at
          # any given distance, to prevent continuous jerky adjustments.
          # BB band should be % of v_ego
          min_vrel_kph_map = OrderedDict([
            # (distance in m, min allowed relative kph)
            (0.5 * safe_dist_m, 3),
            (1.0 * safe_dist_m, -1 - 0.025 * CS.v_ego * CV.MS_TO_KPH),
            (1.5 * safe_dist_m, -5  - 0.05 * CS.v_ego * CV.MS_TO_KPH),
            (3.0 * safe_dist_m, -10 - 0.1 * CS.v_ego * CV.MS_TO_KPH)])
          min_vrel_kph = _interp_map(lead_dist_m, min_vrel_kph_map)
          max_vrel_kph_map = OrderedDict([
            # (distance in m, max allowed relative kph)
            (0.5 * safe_dist_m, 6),
            (1.0 * safe_dist_m, 2),
            (1.5 * safe_dist_m, -3),
            # With visual radar the relative velocity is 0 until the confidence
            # gets high. So even a small negative number here gives constant
            # accel until lead lead car gets close enough to read.
            (3 * safe_dist_m, -7)])
          max_vrel_kph = _interp_map(lead_dist_m, max_vrel_kph_map)
          #if CS.useTeslaRadar:
          #  min_vrel_kph = -1
          #  max_vrel_kph = -2
          min_kph = lead_absolute_speed_kph - max_vrel_kph
          max_kph = lead_absolute_speed_kph - min_vrel_kph
          # In the special case were we are going faster than intended but it's
          # still an acceptable speed, accept it. This could happen if the
          # driver manually accelerates, or if we roll down a hill. In either
          # case, don't fight the extra velocity unless necessary.
          if lead_dist_m >= 0.8 * safe_dist_m and lead_absolute_speed_kph > actual_speed_kph and self.lead_1.aLeadK >= 0.:
            new_speed_kph = lead_absolute_speed_kph
          new_speed_kph =  clip(new_speed_kph, min_kph, max_kph)
          if (actual_speed_kph > new_speed_kph) and (min_kph < actual_speed_kph < max_kph) and (lead_absolute_speed_kph > 30):
            new_speed_kph = actual_speed_kph
          #BB disabled this condition as it might not allow fast enough braking
          #if (actual_speed_kph > 80) and abs(actual_speed_kph - new_speed_kph) < 3.:
          #  new_speed_kph = (actual_speed_kph + new_speed_kph)/2.0
          # Enforce limits on speed in the presence of a lead car.
          new_speed_kph = min(new_speed_kph,
                              _max_safe_speed_kph(self.lead_1,CS),
                              max(lead_absolute_speed_kph - _min_safe_vrel_kph(self.lead_1,CS,actual_speed_kph),2))
    # Enforce limits on speed
    new_speed_kph = clip(new_speed_kph, MIN_PCC_V_KPH, MAX_PCC_V_KPH)
    new_speed_kph = clip(new_speed_kph, MIN_PCC_V_KPH, self.pedal_speed_kph)
    if CS.turn_signal_blinking or (abs(CS.angle_steers) > ANGLE_STOP_ACCEL) or alca_enabled:
      # Don't accelerate during manual turns, curves or ALCA.
      new_speed_kph = min(new_speed_kph, self.last_speed_kph)
    #BB Last safety check. Zero if below MIN_SAFE_DIST_M
    if (MIN_SAFE_DIST_M > lead_dist_m > 0) and (rel_speed_kph < 3.):
      new_speed_kph = MIN_PCC_V_KPH
    self.last_speed_kph = new_speed_kph
    return new_speed_kph * CV.KPH_TO_MS
    
  def pedal_hysteresis(self, pedal, enabled):
    # for small accel oscillations within PEDAL_HYST_GAP, don't change the command
    if not enabled:
      # send 0 when disabled, otherwise acc faults
      self.pedal_steady = 0.
    elif pedal > self.pedal_steady + PEDAL_HYST_GAP:
      self.pedal_steady = pedal - PEDAL_HYST_GAP
    elif pedal < self.pedal_steady - PEDAL_HYST_GAP:
      self.pedal_steady = pedal + PEDAL_HYST_GAP
    return self.pedal_steady

  def _update_pedal_state(self, CS, frame):
    if CS.pedal_idx != CS.prev_pedal_idx:
      # time out pedal after 500ms without receiving a new CAN message from it
      self.pedal_timeout_frame = frame + 50
    self.prev_pcc_available = self.pcc_available
    pedal_ready = frame < self.pedal_timeout_frame and CS.pedal_interceptor_state == 0
    acc_disabled = CS.forcePedalOverCC or CruiseState.is_off(CS.pcm_acc_status)
    # Mark pedal unavailable while traditional cruise is on.
    self.pcc_available = pedal_ready and acc_disabled

    if self.pcc_available != self.prev_pcc_available:
      CS.config_ui_buttons(self.pcc_available, pedal_ready and not acc_disabled)


def _visual_radar_adjusted_dist_m(m, CS):
  # visual radar sucks at short distances. It rarely shows readings below 7m.
  # So rescale distances with 7m -> 0m. Maxes out at 1km, if that matters.
  mapping = OrderedDict([
    # (input distance, output distance)
    (7,    0),      # anything below 7m is set to 0m.
    (1000, 1000)])  # values >7m are scaled, maxing out at 1km.
  return _interp_map(m, mapping)

def _safe_distance_m(v_ego_ms, CS):
  return max(CS.apFollowTimeInS * (v_ego_ms+1), MIN_SAFE_DIST_M)

def _max_safe_speed_kph(lead,CS):
  if _is_present(lead):
    return (CS.v_ego + lead.vRel + (lead.dRel - _safe_distance_m(CS.v_ego,CS))/CS.apFollowTimeInS) * CV.MS_TO_KPH
  else:
    return MAX_PCC_V_KPH
  
def _min_safe_vrel_kph(lead,CS,actual_speed_kph):
  m = lead.dRel
  #BB if lead accelerating do not use this for limit, we have other conditions
  if lead.vLeadK * CV.MS_TO_KPH > actual_speed_kph:
    return -100
  min_vrel_by_distance = OrderedDict([
    # (meters, safe relative velocity in kph)
    # Remember, a negative relative velocity means we are closing the distance.
    (MIN_SAFE_DIST_M, 2),    # If lead is close, it better be pulling away.
    (100,             -25),
    (1000, -50)]) # if lead is far, it's ok if we're closing.
  return _interp_map(m, min_vrel_by_distance)

def _is_present(lead):
  return bool((not (lead is None)) and (lead.dRel > 0))

def _sec_til_collision(lead, CS):
  if _is_present(lead) and lead.vRel < 0:
    if CS.useTeslaRadar:
      #BB: take in consideration acceleration when looking at time to collision. 
      return min(0.1,-4+lead.dRel / abs(lead.vRel + min(0,lead.aRel) * CS.apFollowTimeInS))
    else:
      return _visual_radar_adjusted_dist_m(lead.dRel, CS) / abs(lead.vRel + min(0,lead.aRel) * CS.apFollowTimeInS)
  else:
    return 60.  # Arbitrary, but better than MAXINT because we can still do math on it.
  
def _interp_map(val, val_map):
  """Helper to call interp with an OrderedDict for the mapping. I find
  this easier to read than interp, which takes two arrays."""
  return interp(val, list(val_map.keys()), list(val_map.values()))
  
def _accel_limit_multiplier(CS, lead):
  """Limits acceleration in the presence of a lead car. The further the lead car
  is, the more accel is allowed. Range: 0 to 1, so that it can be multiplied
  with other accel limits."""
  accel_by_speed = OrderedDict([
    # (speed m/s, decel)
      (0.,  0.95),  #   0 kmh
      (10., 0.95),  #  35 kmh
      (20., 0.925),  #  72 kmh
      (30., 0.875)]) # 107 kmh
  if CS.teslaModel in ["SP","SPD"]:
      accel_by_speed = OrderedDict([
        # (speed m/s, decel)
        (0.,  0.95),  #   0 kmh
        (10., 0.95),  #  35 kmh
        (20., 0.925),  #  72 kmh
        (30., 0.875)]) # 107 kmh
  accel_mult = _interp_map(CS.v_ego, accel_by_speed)
  if _is_present(lead):
    safe_dist_m = _safe_distance_m(CS.v_ego,CS)
    accel_multipliers = OrderedDict([
      # (distance in m, acceleration fraction)
      (0.6 * safe_dist_m, 0.15),
      (1.0 * safe_dist_m, 0.2),
      (3.0 * safe_dist_m, 0.4)])
    vrel_multipliers = OrderedDict([
      # vrel m/s, accel mult
      (0. , 1.),
      (10., 1.5)])

    return min(accel_mult * _interp_map(lead.vRel, vrel_multipliers) * _interp_map(lead.dRel, accel_multipliers),1.0)
  else:
    return min(accel_mult * 0.4, 1.0)

def _decel_limit(accel_min,v_ego, lead, CS, max_speed_kph):
  max_speed_mult = 1.
  safe_dist_m = _safe_distance_m(v_ego,CS)
  # if above speed limit quickly decel
  if v_ego * CV.MS_TO_KPH > max_speed_kph:
    overshot = v_ego * CV.MS_TO_KPH - max_speed_kph
    if overshot >= 5:
      max_speed_mult = 2.
    elif overshot >= 2.:
      max_speed_mult = 1.5
  if _is_present(lead):
    time_to_brake = max(0.1,_sec_til_collision(lead, CS))
    if 0 < lead.dRel < MIN_SAFE_DIST_M:
      return -100.
    elif lead.vRel >= 0.1  * v_ego and lead.aRel < 0.5 and lead.dRel <= 1.1 * safe_dist_m:
      # going faster but decelerating, reduce with up to the same acceleration
      return -2 + lead.aRel 
    elif lead.vRel <= 0.1 * v_ego and lead.aLeadK < 0.5  and lead.dRel <= 1.1 * safe_dist_m:
      # going slower AND decelerating
      accel_to_compensate = min(3 * lead.vRel / time_to_brake,-0.7)
      return -2 +  lead.aRel + accel_to_compensate
    elif lead.vRel < -0.1 * v_ego and lead.dRel <= 1.1 * safe_dist_m:
      return -3 + 2 * lead.vRel / time_to_brake
    # if we got here, aLeadK >=0 so use the old logic
    decel_map = OrderedDict([
      # (sec to collision, decel)
      (0, 10.0),
      (4, 1.0),
      (7, 0.5),
      (10, 0.3)])
    decel_speed_map = OrderedDict([
      # (m/s, decel)
      (0, 10.0),
      (4, 5.0),
      (7, 2.50),
      (10, 1.0)])
    return accel_min * max_speed_mult * _interp_map(_sec_til_collision(lead, CS), decel_map) * _interp_map(v_ego, decel_speed_map)
  else:
    #BB: if we don't have a lead, don't do full regen to slow down smoother
    return accel_min * 0.5 * max_speed_mult

def _accel_pedal_max(v_ego, v_target, lead, prev_tesla_accel,CS):
  pedal_max = prev_tesla_accel
  if _is_present(lead):
    #we have lead, base on speed and distance
    safe_dist_m = _safe_distance_m(CS.v_ego,CS)
    v_rel = lead.vLeadK - v_ego
    accel_speed_map = OrderedDict([
      # (speed m/s, decel) change in accel (0..1) per second
      (0.,  0.01),  #  0 MPH 
      (1., 0.1),   # 4 MPH
      (5., 0.15),  # 11 MPH
      (30., 0.20)]) # 67 MPH
    accel_distance_map = OrderedDict([
      # (distance in m, acceleration fraction)
      (0.6 * safe_dist_m, 0.3),
      (1.0 * safe_dist_m, 1.0),
      (3.0 * safe_dist_m, 2.0)])
    pedal_max = prev_tesla_accel + _interp_map(safe_dist_m, accel_distance_map) * _interp_map(v_rel, accel_speed_map) * _DT
  else:
    #no lead, do just based on speed
    accel_speed_map = OrderedDict([
      # (speed m/s, decel) change in accel (0..1) per second
      (0.,  0.25),  #  0 MPH 
      (10., 0.15),  # 22 MPH
      (20., 0.12),  # 45 MPH
      (30., 0.10)]) # 67 MPH
    pedal_max = prev_tesla_accel +  _interp_map(v_ego, accel_speed_map) * _DT
  return 1. #pedal_max

def _brake_pedal_min(v_ego, v_target, lead, CS, max_speed_kph):
  #if less than 7 MPH we don't have much left till 5MPH to brake, so full regen
  if v_ego <= 7 * CV.MPH_TO_MS: 
    return -1
  # if above speed limit quickly decel
  if v_ego * CV.MS_TO_KPH > max_speed_kph:
    return -0.8
  speed_delta_perc = 100 * (v_ego - v_target)/v_ego
  brake_perc_map = OrderedDict([
      # (perc change, decel)
      (0., 0.3),
      (1.5, 0.5),
      (5., 0.8),
      (7., 1.0),
      (50., 1.0)])
  brake_mult1 = _interp_map(speed_delta_perc, brake_perc_map)
  brake_mult2 = 0.
  if _is_present(lead):
    safe_dist_m = _safe_distance_m(CS.v_ego,CS)
    brake_distance_map = OrderedDict([
      # (distance in m, decceleration fraction)
      (0.8 * safe_dist_m, 1.),
      (1.0 * safe_dist_m, .6),
      (3.0 * safe_dist_m, .4)])
    brake_mult2 = _interp_map(lead.dRel, brake_distance_map)
  brake_mult = max(brake_mult1, brake_mult2)
  return -brake_mult
    
def _jerk_limits(v_ego, lead, max_speed_kph, lead_last_seen_time_ms, CS):
  # Allow higher accel jerk at low speed, to get started
  accel_jerk_by_speed = OrderedDict([
    # (Speed in m/s, accel jerk)
    (0, 0.18),
    (9, 0.10)])
  accel_jerk = _interp_map(v_ego, accel_jerk_by_speed)

  # prevent high accel jerk near max speed
  near_max_speed_multipliers = OrderedDict([
    # (kph under max speed, accel jerk multiplier)
    (0, 0.01),
    (4, 1.0)])
  near_max_speed_multiplier = _interp_map(max_speed_kph - v_ego * CV.MS_TO_KPH, near_max_speed_multipliers)
  accel_jerk *= near_max_speed_multiplier

  if _is_present(lead):
    # pick decel jerk based on how much time we have til collision
    decel_jerk_map = OrderedDict([
      # (sec to collision, decel jerk)
      (0, -1.00),
      (2, -0.25),
      (4, -0.01),
      (80, -0.001)])
    decel_jerk = _interp_map(_sec_til_collision(lead, CS), decel_jerk_map)
    safe_dist_m = _safe_distance_m(v_ego,CS) 
    distance_multipliers  = OrderedDict([
      # (distance in m, accel jerk)
      (0.8 * safe_dist_m, 0.01),
      (2.8 * safe_dist_m, 1.00)])
    distance_multiplier = _interp_map(lead.dRel, distance_multipliers)
    accel_jerk *= distance_multiplier
    return decel_jerk, accel_jerk
  else:
    # In the absence of a lead car
    decel_jerk = -0.15
    # Limit accel jerk if the lead was only recently lost, to prevent
    # bucking as a lead is intermittently detected.
    time_since_lead_seen_ms = _current_time_millis() - lead_last_seen_time_ms
    time_since_lead_seen_multipliers = OrderedDict([
      # (ms since last lead sighting, accel jerk multiplier)
      (0,    0.1),
      (2000, 1.0)])
    time_since_lead_seen_multiplier = _interp_map(time_since_lead_seen_ms, time_since_lead_seen_multipliers)
    accel_jerk *= time_since_lead_seen_multiplier

    return decel_jerk, accel_jerk
