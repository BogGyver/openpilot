from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_lkas12, \
                                             create_1191, create_1156, \
                                             create_clu11
from selfdrive.car.hyundai.values import Buttons
from selfdrive.can.packer import CANPacker
from selfdrive.car.modules.ALCA_module import ALCAController


# Steer torque limits

class SteerLimitParams:
  STEER_MAX = 250   # 409 is the max
  STEER_DELTA_UP = 3
  STEER_DELTA_DOWN = 7
  STEER_DRIVER_ALLOWANCE = 50
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1

class CarController(object):
  def __init__(self, dbc_name, car_fingerprint, enable_camera):
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint
    self.lkas11_cnt = 0
    self.cnt = 0
    self.last_resume_cnt = 0
    self.enable_camera = enable_camera
    # True when giraffe switch 2 is low and we need to replace all the camera messages
    # otherwise we forward the camera msgs and we just replace the lkas cmd signals
    self.camera_disconnected = False

    self.packer = CANPacker(dbc_name)
    self.ALCA = ALCAController(self,True,False)  # Enabled  True and SteerByAngle only False


  def update(self, sendcan, enabled, CS, actuators, pcm_cancel_cmd, hud_alert):

    if not self.enable_camera:
      return

    ### Steering Torque
    apply_steer = actuators.steer * SteerLimitParams.STEER_MAX

    apply_steer = apply_std_steer_torque_limits(apply_steer, self.apply_steer_last, CS.steer_torque_driver, SteerLimitParams)

    if not enabled:
      apply_steer = 0

    steer_req = 1 if enabled else 0
    
     #update custom UI buttons and alerts
    CS.UE.update_custom_ui()
    if (frame % 1000 == 0):
      CS.cstm_btns.send_button_info()
      CS.UE.uiSetCarEvent(CS.cstm_btns.car_folder,CS.cstm_btns.car_name)

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


    self.apply_steer_last = apply_steer * -alca_steer

    can_sends = []

    self.lkas11_cnt = self.cnt % 0x10
    self.clu11_cnt = self.cnt % 0x10

    if self.camera_disconnected:
      if (self.cnt % 10) == 0:
        can_sends.append(create_lkas12())
      if (self.cnt % 50) == 0:
        can_sends.append(create_1191())
      if (self.cnt % 7) == 0:
        can_sends.append(create_1156())

    can_sends.append(create_lkas11(self.packer, self.car_fingerprint, apply_steer, steer_req, self.lkas11_cnt,
                                   enabled, CS.lkas11, hud_alert, keep_stock=(not self.camera_disconnected)))

    if pcm_cancel_cmd:
      can_sends.append(create_clu11(self.packer, CS.clu11, Buttons.CANCEL))
    elif CS.stopped and (self.cnt - self.last_resume_cnt) > 5:
      self.last_resume_cnt = self.cnt
      can_sends.append(create_clu11(self.packer, CS.clu11, Buttons.RES_ACCEL))

    ### Send messages to canbus
    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())

    self.cnt += 1
