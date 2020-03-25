from selfdrive.car.tesla.speed_utils.fleet_speed import FleetSpeed
from selfdrive.car.tesla.values import CruiseButtons, CruiseState
from selfdrive.config import Conversions as CV
import cereal.messaging as messaging
import sys
import time


class ACCState():
  # Possible states of the ACC system, following the DI_cruiseState naming
  # scheme.
  OFF = 0         # Disabled by UI.
  STANDBY = 1     # Ready to be enaged.
  ENABLED = 2     # Engaged.
  NOT_READY = 9   # Not ready to be engaged due to the state of the car.
  
class _Mode():
  def __init__(self, label, autoresume, state,adaptive):
    self.label = label
    self.autoresume = autoresume
    self.state = state
    self.adaptive = adaptive
    self.next = None
  
class ACCMode():
  # Possible ACC modes, controlling how ACC behaves.
  # This is separate from ACC state. For example, you could
  # have ACC in "Autoresume" mode in "Standby" state.
  FOLLOW = _Mode(label="follow",   autoresume=False, state=ACCState.STANDBY, adaptive=True)
  AUTO =   _Mode(label="auto", autoresume=True,  state=ACCState.STANDBY, adaptive=True)
  PLAINCC = _Mode(label="cc",   autoresume=False, state=ACCState.STANDBY, adaptive=False)
  
  BUTTON_NAME = 'acc'
  BUTTON_ABREVIATION = 'ACC'
  
  # Toggle order: OFF -> ON -> AUTO -> OFF
  _all_modes = [FOLLOW, AUTO,PLAINCC]
  for index, mode in enumerate(_all_modes):
    mode.next = _all_modes[(index + 1) % len(_all_modes)]
    
  # Map labels to modes for fast lookup by label.
  _label_to_mode = {mode.label: mode for mode in _all_modes}
  @ classmethod
  def from_label(cls, label):
    return cls._label_to_mode.get(label, cls.FOLLOW)
      
  @ classmethod
  def labels(cls):
    return [mode.label for mode in cls._all_modes]

def _current_time_millis():
  return int(round(time.time() * 1000))


class ACCController():
  
  # Tesla cruise only functions above 17 MPH
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS
    
  def __init__(self,carcontroller):
    self.CC = carcontroller
    self.human_cruise_action_time = 0
    self.automated_cruise_action_time = 0
    self.radarState = messaging.sub_sock('radarState', conflate=True)
    self.last_update_time = 0
    self.enable_adaptive_cruise = False
    self.prev_enable_adaptive_cruise = False
    # Whether to re-engage automatically after being paused due to low speed or
    # user-initated deceleration.
    self.autoresume = False
    self.adaptive = False
    self.last_brake_press_time = 0
    self.last_cruise_stalk_pull_time = 0
    self.prev_cruise_buttons = CruiseButtons.IDLE
    self.prev_pcm_acc_status = 0
    self.acc_speed_kph = 0.
    self.speed_limit_kph = 0.
    self.prev_speed_limit_kph = 0.
    self.user_has_braked = False
    self.has_gone_below_min_speed = False
    self.fast_decel_time = 0
    self.lead_last_seen_time_ms = 0
    # BB speed for testing
    self.new_speed = 0
    average_speed_over_x_suggestions = 20 # 1 second (20x a second)
    self.fleet_speed = FleetSpeed(average_speed_over_x_suggestions)

  # Updates the internal state of this controller based on user input,
  # specifically the steering wheel mounted cruise control stalk, and OpenPilot
  # UI buttons.
  def update_stat(self, CS, enabled):
    # Check if the cruise stalk was double pulled, indicating that adaptive
    # cruise control should be enabled. Twice in .75 seconds counts as a double
    # pull.
    self.prev_enable_adaptive_cruise = self.enable_adaptive_cruise
    acc_string = CS.cstm_btns.get_button_label2(ACCMode.BUTTON_NAME)
    acc_mode = ACCMode.from_label(acc_string)
    CS.cstm_btns.get_button(ACCMode.BUTTON_NAME).btn_label2 = acc_mode.label
    self.autoresume = acc_mode.autoresume
    self.adaptive = acc_mode.adaptive
    curr_time_ms = _current_time_millis()
    # Handle pressing the enable button.
    if (CS.cruise_buttons == CruiseButtons.MAIN  or CS.cruise_buttons == CruiseButtons.DECEL_SET) and self.prev_cruise_buttons != CS.cruise_buttons:
      double_pull = curr_time_ms - self.last_cruise_stalk_pull_time < 750
      self.last_cruise_stalk_pull_time = curr_time_ms
      ready = (CS.cstm_btns.get_button_status(ACCMode.BUTTON_NAME) > ACCState.OFF
               and (enabled or (CS.cruise_buttons == CruiseButtons.DECEL_SET and not self.adaptive))
               and CruiseState.is_enabled_or_standby(CS.pcm_acc_status)
               and CS.v_ego > self.MIN_CRUISE_SPEED_MS)
      if ready and double_pull and ((self.adaptive and CS.cruise_buttons == CruiseButtons.MAIN) or ((not self.adaptive) and CS.cruise_buttons == CruiseButtons.DECEL_SET)):
        # A double pull enables ACC. updating the max ACC speed if necessary.
        self.enable_adaptive_cruise = True
        # Increase ACC speed to match current, if applicable.
        if self.adaptive:
          self.acc_speed_kph = max(CS.v_ego_raw * CV.MS_TO_KPH, self.speed_limit_kph)
        else:
          self.acc_speed_kph = CS.v_ego_raw * CV.MS_TO_KPH
        self.user_has_braked = False
        self.has_gone_below_min_speed = False
      else:
        # A single pull disables ACC (falling back to just steering).
        if CS.cruise_buttons == CruiseButtons.MAIN:
          self.enable_adaptive_cruise = False
    # Handle pressing the cancel button.
    if CS.cruise_buttons == CruiseButtons.CANCEL:
      self.enable_adaptive_cruise = False
      self.acc_speed_kph = 0. 
      self.last_cruise_stalk_pull_time = 0
    # Handle pressing up and down buttons.
    elif (CS.cruise_buttons != CruiseButtons.MAIN and self.enable_adaptive_cruise and
          CS.cruise_buttons != self.prev_cruise_buttons):
      self._update_max_acc_speed(CS)
      
    if CS.brake_pressed:
      self.user_has_braked = True
      self.last_brake_press_time = _current_time_millis()
      if not self.autoresume:
        self.enable_adaptive_cruise = False
        
    if CS.v_ego < self.MIN_CRUISE_SPEED_MS:
      self.has_gone_below_min_speed = True
    
    # If autoresume is not enabled and not in standard CC, disable if we hit the brakes or gone below 18mph
    if not self.autoresume:
      if (self.adaptive and not enabled) or self.user_has_braked or self.has_gone_below_min_speed:
        self.enable_adaptive_cruise = False
    
    # Notify if ACC was toggled
    if self.prev_enable_adaptive_cruise and not self.enable_adaptive_cruise:
      CS.UE.custom_alert_message(3, "%s Disabled" % ("ACC" if self.adaptive else "CC"), 150, 4)
      CS.cstm_btns.set_button_status(ACCMode.BUTTON_NAME, ACCState.STANDBY)
      self.fleet_speed.reset_averager()
    elif self.enable_adaptive_cruise:
      CS.cstm_btns.set_button_status(ACCMode.BUTTON_NAME, ACCState.ENABLED)
      if not self.prev_enable_adaptive_cruise:
        CS.UE.custom_alert_message(2, "%s Enabled" % ("ACC" if self.adaptive else "CC"), 150)

    # Update the UI to show whether the current car state allows ACC.
    if CS.cstm_btns.get_button_status(ACCMode.BUTTON_NAME) in [ACCState.STANDBY, ACCState.NOT_READY]:
      if ((enabled or not self.adaptive)
          and CruiseState.is_enabled_or_standby(CS.pcm_acc_status)
          and CS.v_ego > self.MIN_CRUISE_SPEED_MS):
        CS.cstm_btns.set_button_status(ACCMode.BUTTON_NAME, ACCState.STANDBY)
      else:
        CS.cstm_btns.set_button_status(ACCMode.BUTTON_NAME, ACCState.NOT_READY)
          
    # Update prev state after all other actions.
    self.prev_cruise_buttons = CS.cruise_buttons
    self.prev_pcm_acc_status = CS.pcm_acc_status
    
  def _update_max_acc_speed(self, CS):
    # Adjust the max ACC speed based on user cruise stalk actions.
    half_press_kph, full_press_kph = self._get_cc_units_kph(CS.imperial_speed_units)
    speed_change_map = {
      CruiseButtons.RES_ACCEL:     half_press_kph,
      CruiseButtons.RES_ACCEL_2ND: full_press_kph,
      CruiseButtons.DECEL_SET:     -1 * half_press_kph,
      CruiseButtons.DECEL_2ND:     -1 * full_press_kph
    }
    self.acc_speed_kph += speed_change_map.get(CS.cruise_buttons, 0)

    # Clip ACC speed between 0 and 170 KPH.
    self.acc_speed_kph = min(self.acc_speed_kph, 170)
    self.acc_speed_kph = max(self.acc_speed_kph, 0)

  # Decide which cruise control buttons to simluate to get the car to the desired speed.
  def update_acc(self, enabled, CS, frame, actuators, pcm_speed, speed_limit_kph, set_speed_limit_active, speed_limit_offset):
    # Adaptive cruise control
    self.prev_speed_limit_kph = self.speed_limit_kph
    if set_speed_limit_active and speed_limit_kph > 0:
      self.speed_limit_kph = speed_limit_kph + speed_limit_offset
      if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph):
        self.acc_speed_kph = self.speed_limit_kph
        self.fleet_speed.reset_averager()
    else: # reset internal speed limit, so double pull doesn't set higher speed than current (e.g. after leaving the highway)
      self.speed_limit_kph = 0.
    current_time_ms = _current_time_millis()
    if CruiseButtons.should_be_throttled(CS.cruise_buttons):
      self.human_cruise_action_time = current_time_ms
    button_to_press = None
    
    # If ACC is disabled, disengage traditional cruise control.
    if((self.prev_enable_adaptive_cruise) and (not self.enable_adaptive_cruise)
        and (CS.pcm_acc_status == CruiseState.ENABLED)):
      button_to_press = CruiseButtons.CANCEL

    #if non addaptive and we just engaged ACC but pcm is not engaged, then engage
    if (not self.adaptive) and self.enable_adaptive_cruise and (CS.pcm_acc_status != CruiseState.ENABLED):
      button_to_press = CruiseButtons.MAIN

    #if plain cc, not adaptive, then just return None or Cancel
    if (not self.adaptive) and self.enable_adaptive_cruise:
      self.acc_speed_kph = CS.v_cruise_actual #if not CS.imperial_speed_units else CS.v_cruise_actual * CV.MPH_TO_KPH
      return button_to_press

    #disengage if cruise is canceled
    if (not self.enable_adaptive_cruise) and (CS.pcm_acc_status >= 2) and (CS.pcm_acc_status <= 4):
      button_to_press = CruiseButtons.CANCEL 
    lead_1 = None
    #if enabled:
    lead = messaging.recv_one_or_none(self.radarState)
    if lead is not None:
        lead_1 = lead.radarState.leadOne
        if lead_1.dRel:
          self.lead_last_seen_time_ms = current_time_ms
    if self.enable_adaptive_cruise and enabled: # and (button_to_press == None):
      if CS.cstm_btns.get_button_label2(ACCMode.BUTTON_NAME) in ["OP", "AutoOP"]:    
        button_to_press = self._calc_button(CS, pcm_speed)
        self.new_speed = pcm_speed * CV.MS_TO_KPH
      else:
        # Alternative speed decision logic that uses the lead car's distance
        # and speed more directly.
        # Bring in the lead car distance from the radarState feed
        
        button_to_press = self._calc_follow_button(CS, lead_1, speed_limit_kph, set_speed_limit_active, speed_limit_offset, frame)
    if button_to_press:
      self.automated_cruise_action_time = current_time_ms
      # If trying to slow below the min cruise speed, just cancel cruise.
      # This prevents a SCCM crash which is triggered by repeatedly pressing
      # stalk-down when already at min cruise speed.
      if (CruiseButtons.is_decel(button_to_press)
          and CS.v_cruise_actual - 1 < self.MIN_CRUISE_SPEED_MS * CV.MS_TO_KPH):
        button_to_press = CruiseButtons.CANCEL
      if button_to_press == CruiseButtons.CANCEL:
        self.fast_decel_time = current_time_ms
      # Debug logging (disable in production to reduce latency of commands)
      #print "***ACC command: %s***" % button_to_press
    return button_to_press

  # function to calculate the cruise button based on a safe follow distance
  def _calc_follow_button(self, CS, lead_car, speed_limit_kph, set_speed_limit_active, speed_limit_offset, frame):
    if lead_car is None:
      return None
    # Desired gap (in seconds) between cars.
    follow_time_s = CS.apFollowTimeInS
    # v_ego is in m/s, so safe_dist_m is in meters.
    safe_dist_m = CS.v_ego * follow_time_s
    current_time_ms = _current_time_millis()
     # Make sure we were able to populate lead_1.
    # dRel is in meters.
    lead_dist_m = lead_car.dRel
    lead_speed_kph = (lead_car.vRel + CS.v_ego) * CV.MS_TO_KPH
    # Relative velocity between the lead car and our set cruise speed.
    future_vrel_kph = lead_speed_kph - CS.v_cruise_actual
    # How much we can accelerate without exceeding the max allowed speed.
    max_acc_speed_kph = self.fleet_speed.adjust(CS, self.acc_speed_kph * CV.KPH_TO_MS, frame) * CV.MS_TO_KPH
    available_speed_kph = max_acc_speed_kph - CS.v_cruise_actual
    half_press_kph, full_press_kph = self._get_cc_units_kph(CS.imperial_speed_units)
    # button to issue
    button = None
    # debug msg
    msg = None

    # Automatically engage traditional cruise if ACC is active.
    if self._should_autoengage_cc(CS, lead_car=lead_car) and self._no_action_for(milliseconds=100):
      button = CruiseButtons.RES_ACCEL
    # If traditional cruise is engaged, then control it.
    elif CS.pcm_acc_status == CruiseState.ENABLED:
      
      # Disengage cruise control if a slow object is seen ahead. This triggers
      # full regen braking, which is stronger than the braking that happens if
      # you just reduce cruise speed.
      if self._fast_decel_required(CS, lead_car) and self._no_human_action_for(milliseconds=500):
        msg = "Off (Slow traffic)"
        button = CruiseButtons.CANCEL
        self.new_speed = 1
        
      # if cruise is set to faster than the max speed, slow down
      elif CS.v_cruise_actual > max_acc_speed_kph and self._no_action_for(milliseconds=300):
        msg =  "Slow to max"
        button = CruiseButtons.DECEL_SET
        self.new_speed =  max_acc_speed_kph 
        
      elif (# if we have a populated lead_distance
            lead_dist_m > 0
            and self._no_action_for(milliseconds=300)
            # and we're moving
            and CS.v_cruise_actual > full_press_kph):
        ### Slowing down ###
        # Reduce speed significantly if lead_dist < safe dist
        # and if the lead car isn't already pulling away.
        if lead_dist_m < safe_dist_m * .5 and future_vrel_kph < 2:
          msg =  "-5 (Significantly too close)"
          button = CruiseButtons.DECEL_2ND
          self.new_speed = CS.v_ego * CV.MS_TO_KPH - full_press_kph
        # Don't rush up to lead car
        elif future_vrel_kph < -15:
          msg =  "-5 (approaching too fast)"
          button = CruiseButtons.DECEL_2ND
          self.new_speed = CS.v_ego * CV.MS_TO_KPH - full_press_kph
        elif future_vrel_kph < -8:
          msg =  "-1 (approaching too fast)"
          button = CruiseButtons.DECEL_SET
          self.new_speed = CS.v_ego * CV.MS_TO_KPH - half_press_kph
        elif lead_dist_m < safe_dist_m and future_vrel_kph <= 0:
          msg =  "-1 (Too close)"
          button = CruiseButtons.DECEL_SET
          self.new_speed = CS.v_ego * CV.MS_TO_KPH - half_press_kph
        # Make slow adjustments if close to the safe distance.
        # only adjust every 1 secs
        elif (lead_dist_m < safe_dist_m * 1.3
              and future_vrel_kph < -1 * half_press_kph
              and self._no_action_for(milliseconds=1000)):
          msg =  "-1 (Near safe distance)"
          button = CruiseButtons.DECEL_SET
          self.new_speed = CS.v_ego * CV.MS_TO_KPH - half_press_kph

        ### Speed up ###
        elif (available_speed_kph > half_press_kph
              and lead_dist_m > safe_dist_m
              and self._no_human_action_for(milliseconds=1000)):
          lead_is_far = lead_dist_m > safe_dist_m * 1.75
          closing = future_vrel_kph < -2
          lead_is_pulling_away = future_vrel_kph > 4
          if lead_is_far and not closing or lead_is_pulling_away:
            msg =  "+1 (Beyond safe distance and speed)"
            button = CruiseButtons.RES_ACCEL
            self.new_speed = CS.v_ego * CV.MS_TO_KPH + half_press_kph
          
      # If lead_dist is reported as 0, no one is detected in front of you so you
      # can speed up. Only accel on straight-aways; vision radar often
      # loses lead car in a turn.
      elif (lead_dist_m == 0
            and CS.angle_steers < 2.0
            and half_press_kph < available_speed_kph
            and self._no_action_for(milliseconds=500)
            and self._no_human_action_for(milliseconds=1000)
            and current_time_ms > self.lead_last_seen_time_ms + 4000):
          msg =  "+1 (road clear)"
          button = CruiseButtons.RES_ACCEL
          self.new_speed = CS.v_ego * CV.MS_TO_KPH + half_press_kph

    if (current_time_ms > self.last_update_time + 1000):
      ratio = 0
      if safe_dist_m > 0:
        ratio = (lead_dist_m / safe_dist_m) * 100
      print ("Ratio: {0:.1f}%  lead: {1:.1f}m  avail: {2:.1f}kph  vRel: {3:.1f}kph  Angle: {4:.1f}deg".format(
        ratio, lead_dist_m, available_speed_kph, lead_car.vRel * CV.MS_TO_KPH, CS.angle_steers))
      self.last_update_time = current_time_ms
      if msg is not None:
        print ("ACC: " + msg)
    return button
    
  def _should_autoengage_cc(self, CS, lead_car=None):
    # Automatically (re)engage cruise control so long as 
    # 1) The carstate allows cruise control
    # 2) There is no imminent threat of collision
    # 3) The user did not cancel ACC by pressing the brake
    cruise_ready = (self.enable_adaptive_cruise
                    and CS.pcm_acc_status == CruiseState.STANDBY
                    and CS.v_ego >= self.MIN_CRUISE_SPEED_MS
                    and _current_time_millis() > self.fast_decel_time + 2000)
                    
    slow_lead = lead_car and lead_car.dRel > 0 and lead_car.vRel < 0 or self._fast_decel_required(CS, lead_car) # pylint: disable=chained-comparison
    
    # "Autoresume" mode allows cruise to engage even after brake events, but
    # shouldn't trigger DURING braking.
    autoresume_ready = (self.autoresume 
			and CS.a_ego >= 0.1 
			and not self.CC.HSO.human_control
			and _current_time_millis() > self.last_brake_press_time + 1000)
    
    braked = self.user_has_braked or self.has_gone_below_min_speed
    
    return cruise_ready and not slow_lead and (autoresume_ready or not braked)
    
  def _fast_decel_required(self, CS, lead_car):
    """ Identifies situations which call for rapid deceleration. """
    if not lead_car or not lead_car.dRel:
      return False

    collision_imminent = self._seconds_to_collision(CS, lead_car) < 4
    
    lead_absolute_speed_ms = lead_car.vRel + CS.v_ego
    lead_too_slow = lead_absolute_speed_ms < self.MIN_CRUISE_SPEED_MS
    
    return collision_imminent or lead_too_slow
    
  def _seconds_to_collision(self, CS, lead_car):
    if not lead_car or not lead_car.dRel:
      return sys.maxsize
    elif lead_car.vRel >= 0:
      return sys.maxsize
    return abs(float(lead_car.dRel) / lead_car.vRel)
    
  def _get_cc_units_kph(self, is_imperial_units):
    # Cruise control buttons behave differently depending on whether the car
    # is configured for metric or imperial units.
    if is_imperial_units:
      # Imperial unit cars adjust cruise in units of 1 and 5 mph.
      half_press_kph = 1 * CV.MPH_TO_KPH
      full_press_kph = 5 * CV.MPH_TO_KPH
    else:
      # Metric cars adjust cruise in units of 1 and 5 kph.
      half_press_kph = 1
      full_press_kph = 5
    return half_press_kph, full_press_kph
    
  # Adjust speed based off OP's longitudinal model. As of OpenPilot 0.5.3, this
  # is inoperable because the planner crashes when given only visual radar
  # inputs. (Perhaps this can be used in the future with a radar install, or if
  # OpenPilot planner changes.)
  def _calc_button(self, CS, desired_speed_ms):
    button_to_press = None
    # Automatically engange traditional cruise if appropriate.
    if self._should_autoengage_cc(CS) and desired_speed_ms >= CS.v_ego:
      button_to_press = CruiseButtons.RES_ACCEL
    # If traditional cruise is engaged, then control it.
    elif (CS.pcm_acc_status == CruiseState.ENABLED
          # But don't make adjustments if a human has manually done so in
          # the last 3 seconds. Human intention should not be overridden.
          and self._no_human_action_for(milliseconds=3000)
          and self._no_automated_action_for(milliseconds=500)):
      # The difference between OP's target speed and the current cruise
      # control speed, in KPH.
      speed_offset_kph = (desired_speed_ms * CV.MS_TO_KPH - CS.v_cruise_actual)
    
      half_press_kph, full_press_kph = self._get_cc_units_kph(CS.imperial_speed_units)
      
      # Reduce cruise speed significantly if necessary. Multiply by a % to
      # make the car slightly more eager to slow down vs speed up.
      if desired_speed_ms < self.MIN_CRUISE_SPEED_MS:
        button_to_press = CruiseButtons.CANCEL
      if speed_offset_kph < -2 * full_press_kph and CS.v_cruise_actual > 0:
        button_to_press = CruiseButtons.CANCEL
      elif speed_offset_kph < -0.6 * full_press_kph and CS.v_cruise_actual > 0:
        # Send cruise stalk dn_2nd.
        button_to_press = CruiseButtons.DECEL_2ND
      # Reduce speed slightly if necessary.
      elif speed_offset_kph < -0.9 * half_press_kph and CS.v_cruise_actual > 0:
        # Send cruise stalk dn_1st.
        button_to_press = CruiseButtons.DECEL_SET
      # Increase cruise speed if possible.
      elif CS.v_ego > self.MIN_CRUISE_SPEED_MS:
        # How much we can accelerate without exceeding max allowed speed.
        available_speed_kph = self.acc_speed_kph - CS.v_cruise_actual
        if speed_offset_kph >= full_press_kph and full_press_kph < available_speed_kph:
          # Send cruise stalk up_2nd.
          button_to_press = CruiseButtons.RES_ACCEL_2ND
        elif speed_offset_kph >= half_press_kph and half_press_kph < available_speed_kph:
          # Send cruise stalk up_1st.
          button_to_press = CruiseButtons.RES_ACCEL
    return button_to_press
    
  def _no_human_action_for(self, milliseconds):
    return _current_time_millis() > self.human_cruise_action_time + milliseconds
    
  def _no_automated_action_for(self, milliseconds):
    return _current_time_millis() > self.automated_cruise_action_time + milliseconds
    
  def _no_action_for(self, milliseconds):
    return self._no_human_action_for(milliseconds) and self._no_automated_action_for(milliseconds)
