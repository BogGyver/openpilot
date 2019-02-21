import os
import subprocess
from  threading import Thread
import traceback
import shlex
from common.params import Params
from collections import namedtuple
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.controls.lib.drive_helpers import rate_limit
from common.numpy_fast import clip, interp
import numpy as np
import math as mth
from common.realtime import sec_since_boot
from selfdrive.car.tesla import teslacan
from selfdrive.car.tesla.values import AH, CruiseButtons, CAR, CM
from selfdrive.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from selfdrive.car.modules.ALCA_module import ALCAController
from selfdrive.car.modules.GYRO_module import GYROController
from selfdrive.car.tesla.ACC_module import ACCController
from selfdrive.car.tesla.PCC_module import PCCController
from selfdrive.car.tesla.HSO_module import HSOController
import zmq
import selfdrive.messaging as messaging
from selfdrive.services import service_list

# Steer angle limits
ANGLE_MAX_BP = [0., 27., 36.]
ANGLE_MAX_V = [410., 92., 36.]

ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .25]     # windup limit
ANGLE_DELTA_VU = [5., 3.5, 0.8]   # unwind limit
#steering adjustment with speed
DES_ANGLE_ADJUST_FACTOR_BP = [0.,13., 44.]
DES_ANGLE_ADJUST_FACTOR = [1.0, 1.0, 1.0]

def process_hud_alert(hud_alert):
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0
  if hud_alert == AH.NONE:          # no alert
    pass
  elif hud_alert == AH.FCW:         # FCW
    fcw_display = hud_alert[1]
  elif hud_alert == AH.STEER:       # STEER
    steer_required = hud_alert[1]
  else:                             # any other ACC alert
    acc_alert = hud_alert[1]

  return fcw_display, steer_required, acc_alert


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "mini_car", "car", "X4",
                      "lanes", "beep", "chime", "fcw", "acc_alert", "steer_required"])


class CarController(object):
  def __init__(self, dbc_name, enable_camera=True):
    self.params = Params()
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.enable_camera = enable_camera
    self.packer = CANPacker(dbc_name)
    self.epas_disabled = True
    self.last_angle = 0.
    self.last_accel = 0.
    self.ALCA = ALCAController(self,True,True)  # Enabled and SteerByAngle both True
    self.ACC = ACCController()
    self.PCC = PCCController(self)
    self.HSO = HSOController(self)
    self.GYRO = GYROController()
    self.sent_DAS_bootID = False
    context = zmq.Context()
    self.poller = zmq.Poller()
    self.speedlimit = messaging.sub_sock(context, service_list['liveMapData'].port, conflate=True, poller=self.poller)
    self.speedlimit_ms = 0
    self.speedlimit_valid = False
    self.speedlimit_units = 0
    self.opState = 0 # 0-disabled, 1-enabled, 2-disabling, 3-unavailable, 5-warning
    self.accPitch = 0.
    self.accRoll = 0.
    self.accYaw = 0.
    self.magPitch = 0.
    self.magRoll = 0.
    self.magYaw = 0.
    self.gyroPitch = 0.
    self.gyroRoll = 0.
    self.gyroYaw = 0.
    self.set_speed_limit_active = False
    self.speed_limit_offset = 0.

  def update(self, sendcan, enabled, CS, frame, actuators, \
             pcm_speed, pcm_override, pcm_cancel_cmd, pcm_accel, \
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert, \
             snd_beep, snd_chime):

    """ Controls thread """

    ## Todo add code to detect Tesla DAS (camera) and go into listen and record mode only (for AP1 / AP2 cars)
    if not self.enable_camera:
      return

    # *** no output if not enabled ***
    if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = True

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    if hud_show_lanes:
      hud_lanes = 1
    else:
      hud_lanes = 0

    # TODO: factor this out better
    if enabled:
      if hud_show_car:
        hud_car = 2
      else:
        hud_car = 1
    else:
      hud_car = 0
    
    # For lateral control-only, send chimes as a beep since we don't send 0x1fa
    #if CS.CP.radarOffCan:

    #print chime, alert_id, hud_alert
    fcw_display, steer_required, acc_alert = process_hud_alert(hud_alert)

    hud = HUDData(int(pcm_accel), int(round(hud_v_cruise)), 1, hud_car,
                  0xc1, hud_lanes, int(snd_beep), snd_chime, fcw_display, acc_alert, steer_required)
 
    if not all(isinstance(x, int) and 0 <= x < 256 for x in hud):
      print "INVALID HUD", hud
      hud = HUDData(0xc6, 255, 64, 0xc0, 209, 0x40, 0, 0, 0, 0)

    # **** process the car messages ****

    # *** compute control surfaces ***

    STEER_MAX = 420
    # Prevent steering while stopped
    MIN_STEERING_VEHICLE_VELOCITY = 0.05 # m/s
    vehicle_moving = (CS.v_ego >= MIN_STEERING_VEHICLE_VELOCITY)
    
    # Basic highway lane change logic
    changing_lanes = CS.right_blinker_on or CS.left_blinker_on

    #upodate custom UI buttons and alerts
    CS.UE.update_custom_ui()
      
    if (frame % 1000 == 0):
      CS.cstm_btns.send_button_info()
      #read speed limit params
      self.set_speed_limit_active = self.params.get("SpeedLimitOffset") is not None #self.params.get("LimitSetSpeed") == "1" and 
      if self.set_speed_limit_active:
        self.speed_limit_offset = float(self.params.get("SpeedLimitOffset"))
      else:
        self.speed_limit_offset = 0.

    #get pitch/roll/yaw every 0.1 sec
    if (frame %10 == 0):
      (self.accPitch, self.accRoll, self.accYaw),(self.magPitch, self.magRoll, self.magYaw),(self.gyroPitch, self.gyroRoll, self.gyroYaw) = self.GYRO.update(CS.v_ego,CS.a_ego,CS.angle_steers)
      CS.UE.uiGyroInfoEvent(self.accPitch, self.accRoll, self.accYaw,self.magPitch, self.magRoll, self.magYaw,self.gyroPitch, self.gyroRoll, self.gyroYaw)

    # Update statuses for custom buttons every 0.1 sec.
    if self.ALCA.pid == None:
      self.ALCA.set_pid(CS)
    if (frame % 10 == 0):
      self.ALCA.update_status(CS.cstm_btns.get_button_status("alca") > 0 and CS.enableALCA)
      #print CS.cstm_btns.get_button_status("alca")

    
    if CS.pedal_interceptor_available:
      #update PCC module info
      self.PCC.update_stat(CS, True, sendcan)
      self.ACC.enable_adaptive_cruise = False
    else:
      # Update ACC module info.
      self.ACC.update_stat(CS, True)
      self.PCC.enable_pedal_cruise = False
    
    # Update HSO module info.
    human_control = False

    # update CS.v_cruise_pcm based on module selected.
    if self.ACC.enable_adaptive_cruise:
      CS.v_cruise_pcm = self.ACC.acc_speed_kph
    elif self.PCC.enable_pedal_cruise:
      CS.v_cruise_pcm = self.PCC.pedal_speed_kph
    else:
      CS.v_cruise_pcm = CS.v_cruise_actual
    # Get the angle from ALCA.
    alca_enabled = False
    turn_signal_needed = 0
    alca_steer = 0.
    apply_angle, alca_steer,alca_enabled, turn_signal_needed = self.ALCA.update(enabled, CS, frame, actuators)
    apply_angle = -apply_angle  # Tesla is reversed vs OP.
    human_control = self.HSO.update_stat(CS, enabled, actuators, frame)
    human_lane_changing = changing_lanes and not alca_enabled
    enable_steer_control = (enabled
                            and not human_lane_changing
                            and not human_control 
                            and  vehicle_moving)
    
    angle_lim = interp(CS.v_ego, ANGLE_MAX_BP, ANGLE_MAX_V)
    apply_angle = clip(apply_angle, -angle_lim, angle_lim)
    # Windup slower.
    if self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle):
      angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_V)
    else:
      angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_VU)

    des_angle_factor = interp(CS.v_ego, DES_ANGLE_ADJUST_FACTOR_BP, DES_ANGLE_ADJUST_FACTOR )
    if alca_enabled or not CS.enableSpeedVariableDesAngle:
      des_angle_factor = 1.
    #BB disable limits to test 0.5.8
    # apply_angle = clip(apply_angle * des_angle_factor, self.last_angle - angle_rate_lim, self.last_angle + angle_rate_lim) 
    # If human control, send the steering angle as read at steering wheel.
    if human_control:
      apply_angle = CS.angle_steers

    # Send CAN commands.
    can_sends = []

    #First we emulate DAS.
    # DAS_longC_enabled (1),DAS_gas_to_resume (1),DAS_apUnavailable (1), DAS_collision_warning (1),  DAS_op_status (4)
    # DAS_speed_kph(8), 
    # DAS_turn_signal_request (2),DAS_forward_collision_warning (2), DAS_hands_on_state (4), 
    # DAS_cc_state (2), DAS_usingPedal(1),DAS_alca_state (5),
    # DAS_acc_speed_limit_mph (8), 
    # DAS_speed_limit_units(8)
    #send fake_das data as 0x553
    # TODO: forward collission warning
    if frame % 10 == 0:
      #get speed limit
      for socket, _ in self.poller.poll(0):
        if socket is self.speedlimit:
          lmd = messaging.recv_one(socket).liveMapData
          self.speedlimit_ms = lmd.speedLimit
          self.speedlimit_valid = lmd.speedLimitValid
          if (self.params.get("IsMetric") == "1"):
            self.speedlimit_units = self.speedlimit_ms * CV.MS_TO_KPH + 0.5
          else:
            self.speedlimit_units = self.speedlimit_ms * CV.MS_TO_MPH + 0.5 
    op_status = 0x02
    hands_on_state = 0x00
    forward_collision_warning = 0 #1 if needed
    if hud_alert == AH.FCW:
      forward_collision_warning = hud_alert[1]
      if forward_collision_warning > 1:
        forward_collision_warning = 1
    #cruise state: 0 unavailable, 1 available, 2 enabled, 3 hold
    cc_state = 1 
    speed_limit_to_car = int(self.speedlimit_units)
    alca_state = 0x00 
    apUnavailable = 0
    gas_to_resume = 0
    collision_warning = 0x00
    acc_speed_limit_mph = 0
    speed_control_enabled = 0
    accel_min = -15
    accel_max = 5
    acc_speed_kph = 0
    if enabled:
      #self.opState  0-disabled, 1-enabled, 2-disabling, 3-unavailable, 5-warning
      if self.opState == 0:
        op_status = 0x02
      if self.opState == 1:
        op_status = 0x03
      if self.opState == 2:
        op_status = 0x08
      if self.opState == 3:
        op_status = 0x01
      if self.opState == 5:
        op_status = 0x03
      alca_state = 0x08 + turn_signal_needed
      #canceled by user
      if self.ALCA.laneChange_cancelled and (self.ALCA.laneChange_cancelled_counter > 0):
        alca_state = 0x14
      #min speed for ALCA
      if CS.CL_MIN_V > CS.v_ego:
        alca_state = 0x05
      if not enable_steer_control:
        #op_status = 0x08
        hands_on_state = 0x02
        apUnavailable = 1
      if hud_alert == AH.STEER:
        if snd_chime == CM.MUTE:
          hands_on_state = 0x03
        else:
          hands_on_state = 0x05
      acc_speed_limit_mph = max(self.ACC.acc_speed_kph * CV.KPH_TO_MPH,1)
      if CS.pedal_interceptor_available:
        acc_speed_limit_mph = max(self.PCC.pedal_speed_kph * CV.KPH_TO_MPH,1)
      if hud_alert == AH.FCW:
        collision_warning = hud_alert[1]
        if collision_warning > 1:
          collision_warning = 1
        #use disabling for alerts/errors to make them aware someting is goin going on
      if (snd_chime == CM.DOUBLE) or (hud_alert == AH.FCW):
        op_status = 0x08
      if self.ACC.enable_adaptive_cruise:
        acc_speed_kph = self.ACC.new_speed #pcm_speed * CV.MS_TO_KPH
      if (CS.pedal_interceptor_available and self.PCC.enable_pedal_cruise) or (self.ACC.enable_adaptive_cruise):
        speed_control_enabled = 1
        cc_state = 2
      else:
        if (CS.pcm_acc_status == 4):
          #car CC enabled but not OP, display the HOLD message
          cc_state = 3
    send_fake_msg = False
    send_fake_warning = False
    if enabled:
      if frame % 2 == 0:
        send_fake_msg = True
      if frame % 25 == 0:
        send_fake_warning = True
    else:
      if frame % 23 == 0:
        send_fake_msg = True
      if frame % 60 == 0:
        send_fake_warning = True
    if send_fake_msg:
      can_sends.append(teslacan.create_fake_DAS_msg(speed_control_enabled,gas_to_resume,apUnavailable, collision_warning, op_status, \
            acc_speed_kph, \
            turn_signal_needed,forward_collision_warning,hands_on_state, \
            cc_state, 1 if (CS.pedal_interceptor_available) else 0,alca_state, \
            acc_speed_limit_mph,
            speed_limit_to_car,
            apply_angle,
            1 if enable_steer_control else 0))
    if send_fake_warning or (self.opState == 2) or (self.opState == 5):
      #if it's time to send OR we have a warning or emergency disable
      can_sends.append(teslacan.create_fake_DAS_warning(CS.DAS_noSeatbelt, CS.DAS_canErrors, \
            CS.DAS_plannerErrors, CS.DAS_doorOpen, CS.DAS_notInDrive, CS.enableDasEmulation, CS.enableRadarEmulation))
    # end of DAS emulation """

    idx = frame % 16
    cruise_btn = None
    if self.ACC.enable_adaptive_cruise and not CS.pedal_interceptor_available:
      cruise_btn = self.ACC.update_acc(enabled, CS, frame, actuators, pcm_speed, \
                    self.speedlimit_ms * CV.MS_TO_KPH,self.speedlimit_valid, \
                    self.set_speed_limit_active, self.speed_limit_offset)
      if cruise_btn:
          cruise_msg = teslacan.create_cruise_adjust_msg(
            spdCtrlLvr_stat=cruise_btn,
            turnIndLvr_Stat= 0, #turn_signal_needed,
            real_steering_wheel_stalk=CS.steering_wheel_stalk)
          # Send this CAN msg first because it is racing against the real stalk.
          can_sends.insert(0, cruise_msg)
    apply_accel = 0.
    if CS.pedal_interceptor_available and frame % 5 == 0: # pedal processed at 20Hz
      apply_accel, accel_needed, accel_idx = self.PCC.update_pdl(enabled, CS, frame, actuators, pcm_speed, \
                    self.speedlimit_ms * CV.MS_TO_KPH, self.speedlimit_valid, \
                    self.set_speed_limit_active, self.speed_limit_offset)
      can_sends.append(teslacan.create_pedal_command_msg(apply_accel, int(accel_needed), accel_idx))
    self.last_angle = apply_angle
    self.last_accel = apply_accel
    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())

