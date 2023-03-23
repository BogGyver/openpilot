#!/usr/bin/env python3

import cereal.messaging as messaging
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.tesla.values import DBC, CAR, CAN_CHASSIS
from ctypes import create_string_buffer
from common.realtime import Ratekeeper
import struct
from cereal import car
from panda import Panda
import time
import sys
from selfdrive.car.modules.CFG_module import save_bool_param,save_float_param,load_bool_param

MAX_PEDAL_ERRORS = 10

m1 = 0.050796813
m2 = 0.101593626
d = -22.85856576

SafetyModel = car.CarParams.SafetyModel

calibrator_statuses = [
  "Initializing...",
  "Configuring Panda...",
  "Reading Pedal Zero...",
  "Detecting Pedal Max...",
  "Detecting Pedal Scale Factor...",
  "Checking Values...",
  "Saving Values...",
  "Calibration Complete",
  "Calibration Error: ",
]

calibration_errors = [
  "Pedal Timeout!",
  "Car Not On!",
  "Car Not In Neutral!",
  "Accelerator pedal pressed!",
  "Brake pedal not pressed!",
]

def _finish_with_error(n):
  print(calibrator_statuses[-1],calibration_errors[n], flush=True)
  sys.exit(1)

def _finish():
  print(calibrator_statuses[-2], flush=True)
  sys.exit(0)

def _show_status(n):
  print(calibrator_statuses[n], flush=True)

def _show_error(n):
  print(calibration_errors[n], flush=True)

def _debug(a_string):
  #print(a_string, flush=True)
  pass

def _current_time_millis():
    return int(round(time.time() * 1000))

#################################################################
# Python implementation so we don't have to depend on boardd
#################################################################
def can_list_to_can_capnp(can_msgs, msgtype='can'):
  dat = messaging.new_message()
  dat.init(msgtype, len(can_msgs))

  for i, can_msg in enumerate(can_msgs):
    if msgtype == 'sendcan':
      cc = dat.sendcan[i]
    else:
      cc = dat.can[i]

    cc.address = can_msg[0]
    cc.busTime = can_msg[1]
    cc.dat = bytes(can_msg[2])
    cc.src = can_msg[3]

  return dat.to_bytes()

def can_capnp_to_can_list(can, src_filter=None):
  ret = []
  for msg in can:
    if src_filter is None or msg.src in src_filter:
      ret.append((msg.address, msg.busTime, msg.dat, msg.src))
  return ret

#################################################################

PEDAL_TIMEOUT = 500 #500ms, 0.5s

class PedalCalibrator:

  @staticmethod
  def checksum(msg_id, dat):
    # TODO: get message ID from name instead
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  

  def get_parser_can0(self):
    # Status messages
    new_signals = [
          ("DI_pedalPos","DI_torque1",0),
          ("DI_gear","DI_torque2",0),
          ("driverBrakeStatus","BrakeMessage",0),
          ("GTW_driveRailReq","GTW_status",0),
    ]
    checks = [
          ('BrakeMessage',1),
          ('DI_torque1',1),
          ('DI_torque2',1),
          ('GTW_status',1),
    ]
    return CANParser(DBC[CAR.PREAP_MODELS]['chassis'], new_signals, checks, CAN_CHASSIS[CAR.PREAP_MODELS])

  def get_parser_can2(self):
    # Pedal messages
    new_signals = [
          ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
          ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0),
          ("STATE", "GAS_SENSOR", 0),
          ("IDX", "GAS_SENSOR", 0),
    ]
    checks = [
          ("GAS_SENSOR", 0),
    ]
    return CANParser(DBC[CAR.PREAP_MODELS]['chassis'], new_signals, checks, 2, enforce_checks=False)
    
  def create_pedal_command_msg(self,accelCommand, enable, pedalcan):
    """Create GAS_COMMAND (0x551) message to comma pedal"""
    msg_id = 0x551
    msg_len = 6
    msg = create_string_buffer(msg_len)
    idx = self.pedal_idx
    if enable == 1:
        int_accelCommand = int((accelCommand - d) / m1)
        int_accelCommand2 = int((accelCommand - d) / m2)
    else:
        int_accelCommand = 0
        int_accelCommand2 = 0
    msg = create_string_buffer(msg_len)
    struct.pack_into(
        "BBBBB",
        msg,
        0,
        int((int_accelCommand >> 8) & 0xFF),
        int_accelCommand & 0xFF,
        int((int_accelCommand2 >> 8) & 0xFF),
        int_accelCommand2 & 0xFF,
        ((enable << 7) + idx) & 0xFF,
    )
    struct.pack_into("B", msg, msg_len - 1, self.checksum(msg_id,msg.raw))
    self.last_pedal_sent_ms = _current_time_millis()
    self.pedal_idx = (self.pedal_idx + 1) % 16
    self.p.can_send(msg_id, msg.raw, pedalcan)
  
  def can_init(self):
    self.p = Panda()
    self.p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  def parse_can_buffer(self,dat):
    ret = []
    for j in range(0, len(dat), 0x10):
      ddat = dat[j:j+0x10]
      f1, f2 = struct.unpack("II", ddat[0:8])
      ret.append((f1 >> 21, f2 >> 16, ddat[8:8 + (f2 & 0xF)], (f2 >> 4) & 0xFF))
    return ret

  def can_recv(self):
    return self.p.can_recv()

  def __init__(self):
    _show_status(0)
    
    self.handle = None
    self.context = None
    self.frame = 0
    self.rate = 1

    self.cp = self.get_parser_can0()
    self.cp2 = self.get_parser_can2()
    self.can_sock = messaging.sub_sock('can')
    self.pm = messaging.PubMaster(['sendcan'])
    self.can_define = CANDefine(DBC[CAR.PREAP_MODELS]['chassis'])

    self.rcv_pedal_idx = -1.
    self.last_rcv_pedal_idx = -1
    self.last_pedal_seen_ms = 0.
    self.last_pedal_sent_ms = 0.
    self.pedal_interceptor_state = 0
    self.pedal_interceptor_value = 1000.
    self.pedal_interceptor_value2 = 1000.
    self.pedal_idx = -1
    self.pedal_available = False
    self.pedal_timeout = True
    self.pedal_can = 2
    if load_bool_param("TinklaPedalCanZero", False):
      self.pedal_can = 0
    self.pedal_error_count = 0

    #for status=2 - computing pedal zero
    self.pedal_zero_count = 0
    self.pedal_zero_values_to_read = 100
    self.pedal_zero_sum = 0
    self.pedal_zero = -1000

    #for status=3 - detecting pedal max
    self.pedal_last_value_sent = 0
    self.pedal_frame_sent = 0
    self.pedal_enabled = 0
    self.pedal_pressed_value = -1000
    self.pedal_max_value = -1000
    self.pedal_step = 0

    #for status=4 - finetuning
    self.finetuning_stage=0
    self.finetuning_target = 99.6
    self.finetuning_step = 0.1
    self.finetuning_sum = 0
    self.finetuning_count = 0
    self.finetuning_steps = 10
    self.finetuning_best_val = 0
    self.finetuning_best_delta = 1000
    self.finetuning_best_result = -1000

    #for status = 5 - validating
    self.validation_stage= 0
    self.validation_target= 0
    self.validation_count = 0
    self.validation_sum = 0
    self.validation_steps= 10
    self.validation_value = 0.

    #final_values
    self.pedal_min = -1000
    self.pedal_max = -1000
    self.pedal_pressed = -1000
    self.pedal_factor = -1000

    self.brakePressed = False
    self.carOn = False

    self.gearShifter = None
    self.gear_neutral = False
    self.di_gas = 0.
    self.status = 0
    self.prev_status = 0
    self.p = None
    _show_status(1)
    self.can_init()

  def show_error(self,n):
    if  self.frame % self.rate == 0:
      _show_error(n)

  def run(self,rate=100):
    self.status = 2
    self.rate = rate
    rk = Ratekeeper(rate) #run at 100Hz
    while 1:
      rk.keep_time()
      self.frame = rk.frame
      if self.status == 7:
        _finish()
      if self.status != self.prev_status:
        _show_status(self.status)
        self.prev_status = self.status
      curr_time_ms = _current_time_millis()
      for addr, _, dat, _ in self.can_recv():
        if addr == 840: #gtw_status
          self.carOn = dat[0] & 0x01
        if addr == 522: #BrakeMessage
          self.brakePressed = ((dat[0] >> 2)& 0x03) != 1
        if addr == 280: #DI_torque2
          self.gearShifter = dat[1] & 0x70
          self.gear_neutral = self.gearShifter == 0x30
        if addr == 264: #DI_torque1
          self.di_gas = dat[6] * 0.4
        if addr == 1362: #GAS_SENSOR
          self.pedal_interceptor_state = (dat[4] >> 7) & 0x01
          self.pedal_interceptor_value = ((dat[0] << 8) + dat[1]) * m1 + d
          self.pedal_interceptor_value2 = ((dat[2] << 8) + dat[3]) * m2 + d
          self.rcv_pedal_idx = (dat[4] & 0x04)
      #Car on
      #self.carOn = bool(self.cp.vl["GTW_status"]["GTW_driveRailReq"] == 1)
      if not self.carOn:
        self.show_error(1)
        continue

      #Brake pressed
      #self.brakePressed = bool(self.cp.vl["BrakeMessage"]["driverBrakeStatus"] != 1)
      if not self.brakePressed:
        self.show_error(4)
        if self.pedal_enabled == 1:
          self.create_pedal_command_msg(0, 0, self.pedal_can)
          self.pedal_enabled = 0
        continue

      #Gear
      #self.gearShifter = GEAR_MAP[self.can_define.dv["DI_torque2"]["DI_gear"].get(int(self.cp.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]
      #self.gear_neutral = self.gearShifter == car.CarState.GearShifter.neutral
      if not self.gear_neutral:
        self.show_error(2)
        if self.pedal_enabled == 1:
          self.create_pedal_command_msg(0, 0, self.pedal_can)
          self.pedal_enabled = 0
        continue

      #DI Pedal Level
      #self.di_gas = self.cp.vl["DI_torque1"]["DI_pedalPos"]
      if self.di_gas > 0 and self.status < 3:
        self.show_error(3)
        if self.pedal_enabled == 1:
          self.create_pedal_command_msg(0, 0, self.pedal_can)
          self.pedal_enabled = 0
        continue

      #Pedal Msg
      #self.pedal_interceptor_state = self.cp2.vl["GAS_SENSOR"]["STATE"]
      #self.pedal_interceptor_value = self.cp2.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"]
      #self.pedal_interceptor_value2 = self.cp2.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]
      #self.rcv_pedal_idx = self.cp2.vl["GAS_SENSOR"]["IDX"]

      if self.rcv_pedal_idx != self.last_rcv_pedal_idx:
        #have new pedal message
        self.last_pedal_seen_ms = curr_time_ms
        self.last_pedal_idx = self.pedal_idx
        #if ok, reset the error counter and process
        if self.pedal_interceptor_state == 0:
          self.pedal_error_count = 0

          #computing pedal zero
          if self.status == 2:
            self.pedal_zero_sum = self.pedal_zero_sum + self.pedal_interceptor_value
            self.pedal_zero_count = self.pedal_zero_count + 1
            if self.pedal_zero_count >= self.pedal_zero_values_to_read:
              self.pedal_min = self.pedal_zero_sum / self.pedal_zero_count
              print("    *** Pedal min detected at " + str(self.pedal_min), flush=True)
              self.pedal_last_value_sent = self.pedal_min
              self.pedal_enabled = 1
              self.status = 3
            else:
              print("    Detecting pedal zero: " + str(self.pedal_zero_count) + "/" + str(self.pedal_zero_values_to_read), flush=True)

          #computing pedal max
          if self.status == 3:
            if self.di_gas >= 99.6 and self.pedal_max_value == -1000 and self.pedal_step == 1:
              print("    *** Pedal max detected at: " + str(self.pedal_last_value_sent))
              self.pedal_max_value = self.pedal_last_value_sent
              self.pedal_last_value_sent = self.pedal_max_value-0.5
              self.finetuning_start = self.pedal_max_value-0.5
              self.pedal_enabled = 1
              self.pedal_step = 0
              self.status = 4
            if self.di_gas > 0 and self.pedal_pressed_value == -1000 and self.pedal_step == 1:
              print("    *** Pedal pressed detected at: " + str(self.pedal_last_value_sent), flush=True)
              self.pedal_pressed_value = self.pedal_last_value_sent
            if self.di_gas < 100:
              print("    Detecting max: sent " + str(self.pedal_last_value_sent) + " detected " + str(self.di_gas), flush=True)
              if self.pedal_step == 1:
                self.pedal_last_value_sent = self.pedal_last_value_sent + 1
                self.pedal_step = 0
              else:
                self.pedal_step = 1

          #detecting scale factor - finetuning values at 0, 50 and 100
          if self.status == 4:
            #detecting and finetuning
            if self.finetuning_count < self.finetuning_steps:
              self.finetuning_sum = self.finetuning_sum + self.di_gas
              self.finetuning_count = self.finetuning_count + 1
            if self.finetuning_count == self.finetuning_steps:
              if abs(self.finetuning_target-self.finetuning_sum/self.finetuning_count) < self.finetuning_best_delta:
                self.finetuning_best_val = self.pedal_last_value_sent
                self.finetuning_best_delta = abs(self.finetuning_target-self.finetuning_sum/self.finetuning_count) 
                self.finetuning_best_result = self.finetuning_sum/self.finetuning_count
              self.pedal_last_value_sent = self.pedal_last_value_sent + self.finetuning_step
              print("   Targeting " + str(self.finetuning_target) + " and got " + str(self.finetuning_sum/self.finetuning_count), flush=True)
              self.finetuning_count = 0
              self.finetuning_sum = 0
              if self.pedal_last_value_sent > self.finetuning_start + 0.5:
                if self.finetuning_stage == 0:
                  self.pedal_max = self.finetuning_best_val
                  self.finetuning_best_val = 0
                  self.finetuning_best_delta = 1000.
                  self.finetuning_stage = 1
                  self.pedal_last_value_sent = self.pedal_pressed_value - 0.5
                  self.finetuning_start = self.pedal_pressed_value - 0.5
                  self.finetuning_target = 0.4
                  print("    Done finetuning pedal max", flush=True)
                else:
                  self.pedal_pressed = self.finetuning_best_val
                  self.pedal_last_value_sent = self.pedal_min
                  self.pedal_factor = 100./(self.pedal_max - self.pedal_pressed)
                  self.status = 5
                  print("    Done finetuning pedal zero", flush=True)

          #verifying values - try to send something to match at every 10 between 0 and 100
          if self.status == 5:
            #verifying
            if self.validation_stage == 0:
              self.validation_stage = 1
              self.validation_target = self.validation_stage * 10
              self.validation_count = 0
              self.validation_sum = 0
              self.validation_value = self.pedal_pressed + self.validation_target * (self.pedal_max- self.pedal_pressed)/100.
              self.pedal_last_value_sent = self.validation_value
            else:
              if self.validation_count < self.validation_steps:
                self.validation_sum = self.validation_sum + self.di_gas
                self.validation_count = self.validation_count + 1
                if self.validation_count == self.validation_steps:
                  print("   Validating " + str(self.validation_target) + " and got " + str(self.validation_sum/self.validation_count), flush=True)
                  self.validation_stage = self.validation_stage + 1
                  if self.validation_stage == 10:
                    print("    Done validating", flush=True)
                    self.status = 6
                  else:
                    self.validation_target = self.validation_stage * 10
                    self.validation_count = 0
                    self.validation_sum = 0
                    self.validation_value = self.pedal_pressed + self.validation_target * (self.pedal_max- self.pedal_pressed)/100.
                    self.pedal_last_value_sent = self.validation_value
          #saving values and exiting
          if self.status == 6:
            #saving
            a=0
            if self.pedal_min != -1000:
              save_float_param("TeslaPedalCalibMin",self.pedal_min)
              a=a+1
              print("Pedal Min: ",self.pedal_min, flush=True)
            if self.pedal_max != -1000:
              save_float_param("TeslaPedalCalibMax",self.pedal_max)
              a=a+1
              print("Pedal Max: ",self.pedal_max, flush=True)
            if self.pedal_pressed != -1000:
              save_float_param("TeslaPedalCalibZero",self.pedal_pressed)
              a=a+1
              print("Pedal Pressed: ",self.pedal_pressed, flush=True)
            if self.pedal_factor != -1000:
              save_float_param("TeslaPedalCalibFactor",self.pedal_factor)
              a=a+1
              print("Pedal Factor: ",self.pedal_factor, flush=True)
            if a == 4:
              save_bool_param("TeslaPedalCalibDone",True)
            self.status = 7
          #keep sending 0 msg to keep pedal happy
          self.create_pedal_command_msg(self.pedal_last_value_sent, self.pedal_enabled, self.pedal_can)
          
      self.pedal_timeout = curr_time_ms - self.last_pedal_seen_ms > PEDAL_TIMEOUT

      self.pedal_available = (
          not self.pedal_timeout
          and 
          self.pedal_interceptor_state == 0
      )

      if self.pedal_timeout or self.pedal_interceptor_state > 0:
        if curr_time_ms - self.last_pedal_sent_ms > PEDAL_TIMEOUT:
          self.pedal_error_count = self.pedal_error_count + 1.
          if self.pedal_error_count > MAX_PEDAL_ERRORS:
            _finish_with_error(0)
          # send reset command
          self.create_pedal_command_msg(0, 0, self.pedal_can)
          _debug("Pedal unavailable (status=" + str(self.pedal_interceptor_state) + "). Resetting...")


if __name__ == "__main__":
  pc = PedalCalibrator()
  pc.run()