from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.toyota.toyotacan import make_can_msg, create_video_target,\
                                           create_steer_command, create_ui_command, \
                                           create_ipas_steer_command, create_accel_command, \
                                           create_fcw_command
from selfdrive.car.toyota.values import ECU, STATIC_MSGS
from selfdrive.can.packer import CANPacker
from selfdrive.car.modules.ALCA_module import ALCAController

VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert

# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 1.5  # 1.5 m/s2
ACCEL_MIN = -3.0 # 3   m/s2
ACCEL_SCALE = max(ACCEL_MAX, -ACCEL_MIN)

# Steer torque limits
class SteerLimitParams:
  STEER_MAX = 1500
  STEER_DELTA_UP = 10       # 1.5s time to peak torque
  STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor

# Steer angle limits (tested at the Crows Landing track and considered ok)
ANGLE_MAX_BP = [0., 5.]
ANGLE_MAX_V = [510., 300.]
ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .15]     # windup limit
ANGLE_DELTA_VU = [5., 3.5, 0.4]   # unwind limit

TARGET_IDS = [0x340, 0x341, 0x342, 0x343, 0x344, 0x345,
              0x363, 0x364, 0x365, 0x370, 0x371, 0x372,
              0x373, 0x374, 0x375, 0x380, 0x381, 0x382,
              0x383]


def accel_hysteresis(accel, accel_steady, enabled):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if not enabled:
    # send 0 when disabled, otherwise acc faults
    accel_steady = 0.
  elif accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady


def process_hud_alert(hud_alert, audible_alert):
  # initialize to no alert
  steer = 0
  fcw = 0
  sound1 = 0
  sound2 = 0

  if hud_alert == VisualAlert.fcw:
    fcw = 1
  elif hud_alert == VisualAlert.steerRequired:
    steer = 1

  if audible_alert == AudibleAlert.chimeWarningRepeat:
    sound1 = 1
  elif audible_alert != AudibleAlert.none:
    # TODO: find a way to send single chimes
    sound2 = 1

  return steer, fcw, sound1, sound2


def ipas_state_transition(steer_angle_enabled, enabled, ipas_active, ipas_reset_counter):

  if enabled and not steer_angle_enabled:
    #ipas_reset_counter = max(0, ipas_reset_counter - 1)
    #if ipas_reset_counter == 0:
    #  steer_angle_enabled = True
    #else:
    #  steer_angle_enabled = False
    #return steer_angle_enabled, ipas_reset_counter
    return True, 0

  elif enabled and steer_angle_enabled:
    if steer_angle_enabled and not ipas_active:
      ipas_reset_counter += 1
    else:
      ipas_reset_counter = 0
    if ipas_reset_counter > 10:  # try every 0.1s
      steer_angle_enabled = False
    return steer_angle_enabled, ipas_reset_counter

  else:
    return False, 0


class CarController(object):
  def __init__(self, dbc_name, car_fingerprint, enable_camera, enable_dsu, enable_apg):
    self.braking = False
    # redundant safety check with the board
    self.controls_allowed = True
    self.last_steer = 0
    self.last_angle = 0
    self.accel_steady = 0.
    self.car_fingerprint = car_fingerprint
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.angle_control = False

    self.steer_angle_enabled = False
    self.ipas_reset_counter = 0
    self.last_fault_frame = -200

    self.fake_ecus = set()
    if enable_camera: self.fake_ecus.add(ECU.CAM)
    if enable_dsu: self.fake_ecus.add(ECU.DSU)
    if enable_apg: self.fake_ecus.add(ECU.APGS)
    self.ALCA = ALCAController(self,True,False)  # Enabled True and SteerByAngle only False

    self.packer = CANPacker(dbc_name)

  def update(self, sendcan, enabled, CS, frame, actuators,
             pcm_cancel_cmd, hud_alert, audible_alert, forwarding_camera, left_line, right_line, lead):
    #update custom UI buttons and alerts
    CS.UE.update_custom_ui()
    if (frame % 1000 == 0):
      CS.cstm_btns.send_button_info()
      CS.UE.uiSetCarEvent(CS.cstm_btns.car_folder,CS.cstm_btns.car_name)
             

    # *** compute control surfaces ***

    # gas and brake
    apply_accel = actuators.gas - actuators.brake
    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady, enabled)
    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)
    # Get the angle from ALCA.
    alca_enabled = False
    alca_steer = 0.
    alca_angle = 0.
    turn_signal_needed = 0
    # Update ALCA status and custom button every 0.1 sec.
    if self.ALCA.pid == None:
      self.ALCA.set_pid(CS)
    if (frame % 10 == 0):
      self.ALCA.update_status(CS.cstm_btns.get_button_status("alca") > 0)
    # steer torque
    alca_angle, alca_steer, alca_enabled, turn_signal_needed = self.ALCA.update(enabled, CS, frame, actuators)
    apply_steer = int(round(alca_steer * SteerLimitParams.STEER_MAX))

    apply_steer = apply_toyota_steer_torque_limits(apply_steer, self.last_steer, CS.steer_torque_motor, SteerLimitParams)

    # only cut torque when steer state is a known fault
    if CS.steer_state in [9, 25]:
      self.last_fault_frame = frame

    # Cut steering for 2s after fault
    if not enabled or (frame - self.last_fault_frame < 200):
      apply_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    self.steer_angle_enabled, self.ipas_reset_counter = \
      ipas_state_transition(self.steer_angle_enabled, enabled, CS.ipas_active, self.ipas_reset_counter)
    #print self.steer_angle_enabled, self.ipas_reset_counter, CS.ipas_active


    # steer angle
    if self.steer_angle_enabled and CS.ipas_active:

      apply_angle = alca_angle
      angle_lim = interp(CS.v_ego, ANGLE_MAX_BP, ANGLE_MAX_V)
      apply_angle = clip(apply_angle, -angle_lim, angle_lim)

      # windup slower
      if self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle):
        angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_V)
      else:
        angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_VU)

      apply_angle = clip(apply_angle, self.last_angle - angle_rate_lim, self.last_angle + angle_rate_lim)
    else:
      apply_angle = CS.angle_steers

    if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = 1

    # on entering standstill, send standstill request
    if CS.standstill and not self.last_standstill:
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_steer = apply_steer
    self.last_angle = apply_angle
    self.last_accel = apply_accel
    self.last_standstill = CS.standstill

    can_sends = []

    #*** control msgs ***
    #print "steer", apply_steer, min_lim, max_lim, CS.steer_torque_motor

    # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    if ECU.CAM in self.fake_ecus:
      if self.angle_control:
        can_sends.append(create_steer_command(self.packer, 0., 0, frame))
      else:
        can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))

    if self.angle_control:
      can_sends.append(create_ipas_steer_command(self.packer, apply_angle, self.steer_angle_enabled,
                                                 ECU.APGS in self.fake_ecus))
    elif ECU.APGS in self.fake_ecus:
      can_sends.append(create_ipas_steer_command(self.packer, 0, 0, True))

    # accel cmd comes from DSU, but we can spam can to cancel the system even if we are using lat only control
    if (frame % 3 == 0 and ECU.DSU in self.fake_ecus) or (pcm_cancel_cmd and ECU.CAM in self.fake_ecus):
      lead = lead or CS.v_ego < 12.    # at low speed we always assume the lead is present do ACC can be engaged
      if ECU.DSU in self.fake_ecus:
        can_sends.append(create_accel_command(self.packer, apply_accel, pcm_cancel_cmd, self.standstill_req, lead))
      else:
        can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead))

    if frame % 10 == 0 and ECU.CAM in self.fake_ecus and not forwarding_camera:
      for addr in TARGET_IDS:
        can_sends.append(create_video_target(frame/10, addr))

    # ui mesg is at 100Hz but we send asap if:
    # - there is something to display
    # - there is something to stop displaying
    alert_out = process_hud_alert(hud_alert, audible_alert)
    steer, fcw, sound1, sound2 = alert_out

    if (any(alert_out) and not self.alert_active) or \
       (not any(alert_out) and self.alert_active):
      send_ui = True
      self.alert_active = not self.alert_active
    else:
      send_ui = False

    if (frame % 100 == 0 or send_ui) and ECU.CAM in self.fake_ecus:
      can_sends.append(create_ui_command(self.packer, steer, sound1, sound2, left_line, right_line))

    if frame % 100 == 0 and ECU.DSU in self.fake_ecus:
      can_sends.append(create_fcw_command(self.packer, fcw))

    #*** static msgs ***

    for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
      if frame % fr_step == 0 and ecu in self.fake_ecus and self.car_fingerprint in cars and not (ecu == ECU.CAM and forwarding_camera):
        # special cases
        if fr_step == 5 and ecu == ECU.CAM and bus == 1:
          cnt = (((frame / 5) % 7) + 1) << 5
          vl = chr(cnt) + vl
        elif addr in (0x489, 0x48a) and bus == 0:
          # add counter for those 2 messages (last 4 bits)
          cnt = ((frame/100)%0xf) + 1
          if addr == 0x48a:
            # 0x48a has a 8 preceding the counter
            cnt += 1 << 7
          vl += chr(cnt)

        can_sends.append(make_can_msg(addr, vl, bus, False))


    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
