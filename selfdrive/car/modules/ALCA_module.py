"""
Copyright 2018-2019 BB Solutions, LLC. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.

  * Neither the name of Google nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

HISTORY
-------
v4.0 - integrated into model_parser.py
v3.6 - moved parameters to carstate.py
v3.5 - changing the start angle to keep turning until we reach MAX_ANGLE_DELTA
v3.4 - read steerRatio from each car parameters file
v3.3 - re-entry logic changed for smoothness
v3.2 - angle adjustment to compesate for road curvature change
v3.1 - new angle logic for a smoother re-entry
v3.0 - better lane dettection logic
v2.0 - detection of lane crossing 
v1.0 - fixed angle move
"""

from common.numpy_fast import interp
from selfdrive.controls.lib.pid import PIController
from common.realtime import sec_since_boot
from selfdrive.services import service_list
import selfdrive.messaging as messaging
import zmq
import numpy as np
from cereal import tesla

#wait time after turn complete before enabling smoother
WAIT_TIME_AFTER_TURN = 2.0

#ALCA
ALCA_line_check_low_limit = 0.25
ALCA_line_check_high_limit = 0.75
ALCA_line_min_prob = 0.01
ALCA_release_distance = 0.3

ALCA_DEBUG = True
DEBUG_INFO = "step {step} of {total_steps}: direction={ALCA_direction} | using visual = {ALCA_use_visual} | over line={ALCA_over_line} | lane width={ALCA_lane_width} | left to move={left_to_move} | from center ={from_center} | C2 offset = {ALCA_OFFSET_C2} | "

class ALCAController(object):
  def __init__(self,carcontroller,alcaEnabled,steerByAngle):
    #import settings
    self.CC = carcontroller  # added to start, will see if we need it actually
    # variables for lane change
    self.angle_offset = 0. #added when one needs to compensate for missalignment
    self.alcaEnabled = alcaEnabled
    self.alca_duration = [2., 3.5, 5.]
    self.laneChange_strStartFactor = 2.
    self.laneChange_strStartMultiplier = 1.5
    self.laneChange_steerByAngle = steerByAngle # steer only by angle; do not call PID
    self.laneChange_last_actuator_angle = 0.
    self.laneChange_last_actuator_delta = 0.
    self.laneChange_last_sent_angle = 0.
    self.laneChange_over_the_line = 0 # did we cross the line?
    self.laneChange_avg_angle = 0. # used if we do average entry angle over x frames
    self.laneChange_avg_count = 0. # used if we do average entry angle over x frames
    self.laneChange_enabled = 1 # set to zero for no lane change
    self.laneChange_counter = 0 # used to count frames during lane change
    self.laneChange_min_duration = 2. # min time to wait before looking for next lane
    self.laneChange_duration = 5.6 # how many max seconds to actually do the move; if lane not found after this then send error
    self.laneChange_after_lane_duration_mult = 1.  # multiplier for time after we cross the line before we let OP take over; multiplied with CL_TIMEA_T 
    self.laneChange_wait = 1 # how many seconds to wait before it starts the change
    self.laneChange_lw = 3.7 # lane width in meters
    self.laneChange_angle = 0. # saves the last angle from actuators before lane change starts
    self.laneChange_angled = 0. # angle delta
    self.laneChange_steerr = 15.75 # steer ratio for lane change starting with the Tesla one
    self.laneChange_direction = 0 # direction of the lane change 
    self.prev_right_blinker_on = False # local variable for prev position
    self.prev_left_blinker_on = False # local variable for prev position
    self.keep_angle = False #local variable to keep certain angle delta vs. actuator
    self.last10delta = []
    self.laneChange_cancelled = False
    self.laneChange_cancelled_counter = 0
    self.last_time_enabled = 0


  def update_status(self,alcaEnabled):
    self.alcaEnabled = alcaEnabled


  def stop_ALCA(self, CS):
    # something is not right; ALCAModelParser is not engaged; cancel
    CS.UE.custom_alert_message(3,"Auto Lane Change Canceled! (d)",200,5)
    self.laneChange_cancelled = True
    self.laneChange_cancelled_counter = 200
    self.laneChange_enabled = 1
    self.laneChange_counter = 0
    self.laneChange_direction = 0
    CS.cstm_btns.set_button_status("alca",1)


  def update(self,enabled,CS,actuators):
    cl_min_v = CS.CL_MIN_V
    cl_max_a = CS.CL_MAX_A
    alca_mode = CS.cstm_btns.get_button_label2_index("alca")

    if self.laneChange_cancelled_counter > 0:
      self.laneChange_cancelled_counter -= 1
      if self.laneChange_cancelled_counter == 0:
        self.laneChange_cancelled = False

    # Basic highway lane change logic
    actuator_delta = 0.
    laneChange_angle = 0.
    turn_signal_needed = 0 # send 1 for left, 2 for right 0 for not needed

    if (not CS.right_blinker_on) and (not CS.left_blinker_on) and \
    (self.laneChange_enabled == 4):
        self.laneChange_enabled =1
        self.laneChange_counter =0
        self.laneChange_direction =0
        CS.UE.custom_alert_message(-1,"",0)
    
    if (not CS.right_blinker_on) and (not CS.left_blinker_on) and \
      (self.laneChange_enabled > 1):
      # no blinkers on but we are still changing lane, so we need to send blinker command
      if self.laneChange_direction == -1:
        turn_signal_needed = 1
      elif self.laneChange_direction == 1:
        turn_signal_needed = 2
      else:
        turn_signal_needed = 0

    if (CS.cstm_btns.get_button_status("alca") > 0) and self.alcaEnabled and (self.laneChange_enabled == 1):
      if ((CS.v_ego < cl_min_v) or (abs(actuators.steerAngle) >= cl_max_a) or \
      (abs(CS.angle_steers)>= cl_max_a)  or (not enabled)): 
        CS.cstm_btns.set_button_status("alca",9)
      else:
        CS.cstm_btns.set_button_status("alca",1)

    if self.alcaEnabled and enabled and (((not self.prev_right_blinker_on) and CS.right_blinker_on) or \
      ((not self.prev_left_blinker_on) and CS.left_blinker_on)) and \
      ((CS.v_ego < cl_min_v) or (abs(actuators.steerAngle) >= cl_max_a) or (abs(CS.angle_steers) >=cl_max_a)):
      # something is not right, the speed or angle is limitting
      CS.UE.custom_alert_message(3,"Auto Lane Change Unavailable!",500,3)
      CS.cstm_btns.set_button_status("alca",9)


    if self.alcaEnabled and enabled and (((not self.prev_right_blinker_on) and CS.right_blinker_on) or \
      ((not self.prev_left_blinker_on) and CS.left_blinker_on))  and \
      (CS.v_ego >= cl_min_v) and (abs(actuators.steerAngle) < cl_max_a):
      # start blinker, speed and angle is within limits, let's go
      laneChange_direction = 1
      # changing lanes
      if CS.left_blinker_on:
        laneChange_direction = -1

      if (self.laneChange_enabled > 1) and (self.laneChange_direction <> laneChange_direction):
        # something is not right; signal in oposite direction; cancel
        CS.UE.custom_alert_message(3,"Auto Lane Change Canceled! (s)",200,5)
        self.laneChange_cancelled = True
        self.laneChange_cancelled_counter = 200
        self.laneChange_enabled = 1
        self.laneChange_counter = 0
        self.laneChange_direction = 0
        CS.cstm_btns.set_button_status("alca",1)
      elif (self.laneChange_enabled == 1) :
        # compute angle delta for lane change
        CS.UE.custom_alert_message(2,"Auto Lane Change Engaged!",100)
        self.laneChange_enabled = 2
        self.laneChange_counter = 1
        self.laneChange_direction = laneChange_direction
        CS.cstm_btns.set_button_status("alca",2)

    if (not self.alcaEnabled) and self.laneChange_enabled > 1:
      self.laneChange_enabled = 1
      self.laneChange_counter = 0
      self.laneChange_direction = 0

    # lane change in progress
    if self.laneChange_enabled > 1:
      if (CS.steer_override or (CS.v_ego < cl_min_v)):
        CS.UE.custom_alert_message(4,"Auto Lane Change Canceled! (u)",200,3)
        self.laneChange_cancelled = True
        self.laneChange_cancelled_counter = 200
        # if any steer override cancel process or if speed less than min speed
        self.laneChange_counter = 0
        self.laneChange_enabled = 1
        self.laneChange_direction = 0
        CS.cstm_btns.set_button_status("alca",1)
      if self.laneChange_enabled == 2:
        if self.laneChange_counter == 1:
          CS.UE.custom_alert_message(2,"Auto Lane Change Engaged! (1)",self.laneChange_wait * 100)
        self.laneChange_counter += 1
        if self.laneChange_counter == self.laneChange_wait * 100:
          self.laneChange_enabled = 3
          self.laneChange_counter = 0
      if self.laneChange_enabled ==3:
        if self.laneChange_counter == 1:
          CS.UE.custom_alert_message(2,"Auto Lane Change Engaged! (2)",int(self.alca_duration[alca_mode] * 100))
        self.laneChange_counter += 1
        if self.laneChange_counter >= self.alca_duration[alca_mode] * 100:
          self.laneChange_enabled = 4
          self.laneChange_counter = 0
      if self.laneChange_enabled == 4:
        if self.laneChange_counter == 1:
          CS.UE.custom_alert_message(2,"Auto Lane Change Complete!",100)
          self.laneChange_enabled = 1
          self.laneChange_counter = 0

    CS.ALCA_enabled = (self.laneChange_enabled > 1) and self.alcaEnabled
    CS.ALCA_total_steps = int(20 * self.alca_duration[alca_mode])
    if self.laneChange_enabled == 3:
      CS.ALCA_direction = -self.laneChange_direction
    else:
      CS.ALCA_direction = 0

    return turn_signal_needed, self.laneChange_enabled > 1

class ALCAModelParser(object):
  def __init__(self):
    #ALCA params
    self.ALCA_error = False
    self.ALCA_lane_width = 3.6
    self.ALCA_direction = 0 # left 1, right -1
    self.ALCA_step = 0
    self.ALCA_total_steps = 20 * 5 #20 Hz, 5 seconds, wifey mode
    self.ALCA_cancelling = False
    self.ALCA_enabled = False
    self.ALCA_OFFSET_C3 = 0.
    self.ALCA_OFFSET_C2 = 0.
    self.ALCA_over_line = False
    self.prev_CS_ALCA_error = False
    self.ALCA_use_visual = True
    self.ALCA_vego = 0.
    self.ALCA_vego_prev = 0.
    self.poller = zmq.Poller()
    self.alcaStatus = messaging.sub_sock(service_list['alcaStatus'].port, conflate=True, poller=self.poller)
    self.alcaState = messaging.pub_sock(service_list['alcaState'].port)
    self.alcas = None


  def reset_alca (self):
    self.ALCA_step = 0
    self.ALCA_direction = 0
    self.ALCA_cancelling = False
    self.ALCA_error = True
    self.ALCA_enabled = False
    self.ALCA_OFFSET_C3 = 0.
    self.ALCA_OFFSET_C2 = 0.
    self.ALCA_over_line = False
    self.ALCA_use_visual = True
    self.ALCA_vego_prev = 0.
    self.alcas = None

  def debug_alca(self,message):
    if ALCA_DEBUG:
      print message

  def send_state(self):
    alca_state = tesla.ALCAState.new_message()
    #ALCA params
    alca_state.alcaDirection = int(self.ALCA_direction)
    alca_state.alcaError = bool(self.ALCA_error)
    alca_state.alcaCancelling = bool(self.ALCA_cancelling)
    alca_state.alcaEnabled = bool(self.ALCA_enabled)
    alca_state.alcaLaneWidth = float(self.ALCA_lane_width)
    alca_state.alcaStep = int(self.ALCA_step)
    alca_state.alcaTotalSteps = int(self.ALCA_total_steps)
    self.alcaState.send(alca_state.to_bytes())

  def update(self, v_ego, md, r_poly, l_poly, r_prob, l_prob, lane_width, p_poly):

    for socket, _ in self.poller.poll(0):
      if socket is self.alcaStatus:
        self.alcas = tesla.ALCAStatus.from_bytes(socket.recv())

    #if we don't have yet ALCA status, return same values
    if self.alcas is None:
      self.send_state()
      return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width, p_poly

     
    self.ALCA_direction = self.alcas.alcaDirection
    self.ALCA_enabled = self.alcas.alcaEnabled
    self.ALCA_total_steps = self.alcas.alcaTotalSteps
    self.ALCA_error = self.ALCA_error or (self.alcas.alcaError and not self.prev_CS_ALCA_error)
    self.prev_CS_ALCA_error = self.alcas.alcaError

    if not self.ALCA_enabled:
      self.send_state()
      return np.array(r_poly),np.array(l_poly),r_prob, l_prob, lane_width, p_poly

    #if error but no direction, the carcontroller component is fine and we need to reset
    if self.ALCA_error and (self.ALCA_direction == 0):
      self.ALCA_error = False


    #where are we in alca as %
    ALCA_perc_complete = float(self.ALCA_step) / float(self.ALCA_total_steps)
    if self.ALCA_error and self.ALCA_cancelling:
      self.debug_alca(" Error and Cancelling -> resetting...")
      self.reset_alca()
    if self.ALCA_error and not self.ALCA_cancelling:
      if (ALCA_perc_complete < 0.1) or (ALCA_perc_complete > 0.9):
        self.debug_alca(" Error and less than 10% -> resetting...")
        self.reset_alca()
      else:
        self.debug_alca(" Error and not Cancelling -> rewinding...")
        self.ALCA_cancelling = True
        self.ALCA_error = False

    if self.ALCA_enabled and not (self.ALCA_direction == 0):
      if ALCA_DEBUG:
        print ALCA_perc_complete, self.ALCA_step,self.ALCA_total_steps
      ALCA_increment = -3 if self.ALCA_cancelling else 1
      self.ALCA_step += ALCA_increment
      if (self.ALCA_step < 0) or (self.ALCA_step >= self.ALCA_total_steps):
        #done so end ALCA
        self.debug_alca(" step out of bounds -> resetting...")
        self.reset_alca()
      else:
        #if between 20% and 80% of change is done, let's check if we are over the line
        if ALCA_line_check_low_limit  < ALCA_perc_complete  < ALCA_line_check_high_limit :
          if self.ALCA_direction == -1:
            #if we are moving to the right
            if (l_prob > ALCA_line_min_prob ) and (0. <= l_poly[3] <=  (self.ALCA_lane_width /2.)):
              self.ALCA_over_line = True
          if self.ALCA_direction == 1:
            #if we are moving to the left
            if (r_prob > ALCA_line_min_prob ) and ((-self.ALCA_lane_width / 2.) <= r_poly[3] <= 0 ):
              self.ALCA_over_line = True
        elif ALCA_perc_complete >= ALCA_line_check_high_limit :
          self.ALCA_over_line = True
        else:
          self.ALCA_over_line = False
        #make sure we always have the line we need in sight
        prev_ALCA_use_visual = self.ALCA_use_visual
        if (not self.ALCA_over_line) and (((self.ALCA_direction == 1) and (l_prob < ALCA_line_min_prob)) or ((self.ALCA_direction == -1) and (r_prob < ALCA_line_min_prob))):
          self.ALCA_use_visual = False
        elif self.ALCA_over_line and (((self.ALCA_direction == 1) and (r_prob < ALCA_line_min_prob)) or ((self.ALCA_direction == -1) and (l_prob < ALCA_line_min_prob))):
          self.ALCA_use_visual = False
        else:
          self.ALCA_use_visual = True

        #did we just switch between visual and non-visual?
        if prev_ALCA_use_visual != self.ALCA_use_visual:
          self.reset_alca()

        #compute offset
        from_center = 0.
        left_to_move = 0.
        if self.ALCA_enabled and not (self.ALCA_direction == 0):
          if self.ALCA_over_line:
            if self.ALCA_direction == 1:
              from_center = self.ALCA_lane_width / 2 - r_poly[3]
            else:
              from_center = self.ALCA_lane_width / 2 + l_poly[3]
          else:
            if self.ALCA_direction == 1:
              from_center = self.ALCA_lane_width / 2 - l_poly[3]
            else:
              from_center = self.ALCA_lane_width / 2 + r_poly[3]
          if from_center < 0.:
            from_center += self.ALCA_lane_width /2 
          left_to_move = self.ALCA_lane_width - from_center
          steps_left = self.ALCA_total_steps - self.ALCA_step
          self.ALCA_OFFSET_C2 = float(self.ALCA_direction * left_to_move) / float(steps_left * 0.05 * (self.ALCA_vego_prev + v_ego) / 2.)
          if ALCA_DEBUG:
            debug_string = DEBUG_INFO.format(step=self.ALCA_step,total_steps=self.ALCA_total_steps,ALCA_direction=self.ALCA_direction,ALCA_use_visual=self.ALCA_use_visual,ALCA_over_line=self.ALCA_over_line,ALCA_lane_width=self.ALCA_lane_width, left_to_move=left_to_move, from_center=from_center, ALCA_OFFSET_C2=self.ALCA_OFFSET_C2)
            self.debug_alca(debug_string)
        else:
          self.ALCA_OFFSET_C2 = 0.
        
        if (not self.ALCA_error) and self.ALCA_use_visual:
          if self.ALCA_over_line:
            if (self.ALCA_total_steps - self.ALCA_step <= 1) or (self.ALCA_over_line and ((self.ALCA_direction == 1) and (r_poly[3] < -ALCA_release_distance)) or ((self.ALCA_direction == -1) and (l_poly[3] > ALCA_release_distance))):
              self.reset_alca()
              self.ALCA_error = False

        if (self.ALCA_direction == 1 and not self.ALCA_over_line) or (self.ALCA_direction == -1 and self.ALCA_over_line):
          r_poly = np.array(l_poly)
          l_prob = 1
          r_prob = l_prob
        elif (self.ALCA_direction == -1 and not self.ALCA_over_line) or (self.ALCA_direction == 1 and self.ALCA_over_line):
          l_poly = np.array(r_poly)
          r_prob = 1
          l_prob = r_prob
        l_poly[3] = self.ALCA_lane_width / 2
        r_poly[3] = -self.ALCA_lane_width / 2
        p_poly[3] = 0
        l_poly[2] += self.ALCA_OFFSET_C2
        r_poly[2] += self.ALCA_OFFSET_C2
        p_poly[2] += self.ALCA_OFFSET_C2
    else:
      self.reset_alca()
      self.ALCA_error = False

    self.ALCA_vego_prev = v_ego

    if self.ALCA_enabled:
      if self.ALCA_direction == 0:
        self.ALCA_lane_width = lane_width
      else:
        lane_width = self.ALCA_lane_width

    self.send_state()
    return np.array(r_poly),np.array(l_poly),r_prob, l_prob, self.ALCA_lane_width, p_poly
 
