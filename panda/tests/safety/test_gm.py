#!/usr/bin/env python3
import unittest
import numpy as np
from panda import Panda
from panda.tests.safety import libpandasafety_py
from panda.tests.safety.common import StdTest, make_msg

MAX_RATE_UP = 7
MAX_RATE_DOWN = 17
MAX_STEER = 300
MAX_BRAKE = 350
MAX_GAS = 3072
MAX_REGEN = 1404

MAX_RT_DELTA = 128
RT_INTERVAL = 250000

DRIVER_TORQUE_ALLOWANCE = 50;
DRIVER_TORQUE_FACTOR = 4;

TX_MSGS = [[384, 0], [1033, 0], [1034, 0], [715, 0], [880, 0],  # pt bus
           [161, 1], [774, 1], [776, 1], [784, 1],  # obs bus
           [789, 2],  # ch bus
           [0x104c006c, 3], [0x10400060]]  # gmlan

def twos_comp(val, bits):
  if val >= 0:
    return val
  else:
    return (2**bits) + val

def sign(a):
  if a > 0:
    return 1
  else:
    return -1

class TestGmSafety(unittest.TestCase):
  @classmethod
  def setUp(cls):
    cls.safety = libpandasafety_py.libpandasafety
    cls.safety.set_safety_hooks(Panda.SAFETY_GM, 0)
    cls.safety.init_tests_gm()

  def _speed_msg(self, speed):
    to_send = make_msg(0, 842)
    to_send[0].RDLR = speed
    return to_send

  def _button_msg(self, buttons):
    to_send = make_msg(0, 481)
    to_send[0].RDHR = buttons << 12
    return to_send

  def _brake_msg(self, brake):
    to_send = make_msg(0, 241)
    to_send[0].RDLR = 0xa00 if brake else 0x900
    return to_send

  def _gas_msg(self, gas):
    to_send = make_msg(0, 417)
    to_send[0].RDHR = (1 << 16) if gas else 0
    return to_send

  def _send_brake_msg(self, brake):
    to_send = make_msg(2, 789)
    brake = (-brake) & 0xfff
    to_send[0].RDLR = (brake >> 8) | ((brake &0xff) << 8)
    return to_send

  def _send_gas_msg(self, gas):
    to_send = make_msg(0, 715)
    to_send[0].RDLR = ((gas & 0x1f) << 27) | ((gas & 0xfe0) << 11)
    return to_send

  def _set_prev_torque(self, t):
    self.safety.set_gm_desired_torque_last(t)
    self.safety.set_gm_rt_torque_last(t)

  def _torque_driver_msg(self, torque):
    t = twos_comp(torque, 11)
    to_send = make_msg(0, 388)
    to_send[0].RDHR = (((t >> 8) & 0x7) << 16) | ((t & 0xFF) << 24)
    return to_send

  def _torque_msg(self, torque):
    t = twos_comp(torque, 11)
    to_send = make_msg(0, 384)
    to_send[0].RDLR = ((t >> 8) & 0x7) | ((t & 0xFF) << 8)
    return to_send

  def test_spam_can_buses(self):
    StdTest.test_spam_can_buses(self, TX_MSGS)

  def test_relay_malfunction(self):
    StdTest.test_relay_malfunction(self, 384)

  def test_default_controls_not_allowed(self):
    self.assertFalse(self.safety.get_controls_allowed())

  def test_resume_button(self):
    RESUME_BTN = 2
    self.safety.set_controls_allowed(0)
    self.safety.safety_rx_hook(self._button_msg(RESUME_BTN))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_set_button(self):
    SET_BTN = 3
    self.safety.set_controls_allowed(0)
    self.safety.safety_rx_hook(self._button_msg(SET_BTN))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_cancel_button(self):
    CANCEL_BTN = 6
    self.safety.set_controls_allowed(1)
    self.safety.safety_rx_hook(self._button_msg(CANCEL_BTN))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_disengage(self):
    StdTest.test_allow_brake_at_zero_speed(self)
    StdTest.test_not_allow_brake_when_moving(self, 0)

  def test_disengage_on_gas(self):
    self.safety.set_controls_allowed(1)
    self.safety.safety_rx_hook(self._gas_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._gas_msg(False))

  def test_allow_engage_with_gas_pressed(self):
    self.safety.safety_rx_hook(self._gas_msg(True))
    self.safety.set_controls_allowed(1)
    self.safety.safety_rx_hook(self._gas_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._gas_msg(False))

  def test_brake_safety_check(self):
    for enabled in [0, 1]:
      for b in range(0, 500):
        self.safety.set_controls_allowed(enabled)
        if abs(b) > MAX_BRAKE or (not enabled and b != 0):
          self.assertFalse(self.safety.safety_tx_hook(self._send_brake_msg(b)))
        else:
          self.assertTrue(self.safety.safety_tx_hook(self._send_brake_msg(b)))

  def test_gas_safety_check(self):
    for enabled in [0, 1]:
      for g in range(0, 2**12-1):
        self.safety.set_controls_allowed(enabled)
        if abs(g) > MAX_GAS or (not enabled and g != MAX_REGEN):
          self.assertFalse(self.safety.safety_tx_hook(self._send_gas_msg(g)))
        else:
          self.assertTrue(self.safety.safety_tx_hook(self._send_gas_msg(g)))

  def test_steer_safety_check(self):
    for enabled in [0, 1]:
      for t in range(-0x200, 0x200):
        self.safety.set_controls_allowed(enabled)
        self._set_prev_torque(t)
        if abs(t) > MAX_STEER or (not enabled and abs(t) > 0):
          self.assertFalse(self.safety.safety_tx_hook(self._torque_msg(t)))
        else:
          self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(t)))

  def test_manually_enable_controls_allowed(self):
    StdTest.test_manually_enable_controls_allowed(self)

  def test_non_realtime_limit_up(self):
    self.safety.set_gm_torque_driver(0, 0)
    self.safety.set_controls_allowed(True)

    self._set_prev_torque(0)
    self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(MAX_RATE_UP)))
    self._set_prev_torque(0)
    self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(-MAX_RATE_UP)))

    self._set_prev_torque(0)
    self.assertFalse(self.safety.safety_tx_hook(self._torque_msg(MAX_RATE_UP + 1)))
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertFalse(self.safety.safety_tx_hook(self._torque_msg(-MAX_RATE_UP - 1)))

  def test_non_realtime_limit_down(self):
    self.safety.set_gm_torque_driver(0, 0)
    self.safety.set_controls_allowed(True)

  def test_against_torque_driver(self):
    self.safety.set_controls_allowed(True)

    for sign in [-1, 1]:
      for t in np.arange(0, DRIVER_TORQUE_ALLOWANCE + 1, 1):
        t *= -sign
        self.safety.set_gm_torque_driver(t, t)
        self._set_prev_torque(MAX_STEER * sign)
        self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(MAX_STEER * sign)))

      self.safety.set_gm_torque_driver(DRIVER_TORQUE_ALLOWANCE + 1, DRIVER_TORQUE_ALLOWANCE + 1)
      self.assertFalse(self.safety.safety_tx_hook(self._torque_msg(-MAX_STEER)))

    # spot check some individual cases
    for sign in [-1, 1]:
      driver_torque = (DRIVER_TORQUE_ALLOWANCE + 10) * sign
      torque_desired = (MAX_STEER - 10 * DRIVER_TORQUE_FACTOR) * sign
      delta = 1 * sign
      self._set_prev_torque(torque_desired)
      self.safety.set_gm_torque_driver(-driver_torque, -driver_torque)
      self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(torque_desired)))
      self._set_prev_torque(torque_desired + delta)
      self.safety.set_gm_torque_driver(-driver_torque, -driver_torque)
      self.assertFalse(self.safety.safety_tx_hook(self._torque_msg(torque_desired + delta)))

      self._set_prev_torque(MAX_STEER * sign)
      self.safety.set_gm_torque_driver(-MAX_STEER * sign, -MAX_STEER * sign)
      self.assertTrue(self.safety.safety_tx_hook(self._torque_msg((MAX_STEER - MAX_RATE_DOWN) * sign)))
      self._set_prev_torque(MAX_STEER * sign)
      self.safety.set_gm_torque_driver(-MAX_STEER * sign, -MAX_STEER * sign)
      self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(0)))
      self._set_prev_torque(MAX_STEER * sign)
      self.safety.set_gm_torque_driver(-MAX_STEER * sign, -MAX_STEER * sign)
      self.assertFalse(self.safety.safety_tx_hook(self._torque_msg((MAX_STEER - MAX_RATE_DOWN + 1) * sign)))


  def test_realtime_limits(self):
    self.safety.set_controls_allowed(True)

    for sign in [-1, 1]:
      self.safety.init_tests_gm()
      self._set_prev_torque(0)
      self.safety.set_gm_torque_driver(0, 0)
      for t in np.arange(0, MAX_RT_DELTA, 1):
        t *= sign
        self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(t)))
      self.assertFalse(self.safety.safety_tx_hook(self._torque_msg(sign * (MAX_RT_DELTA + 1))))

      self._set_prev_torque(0)
      for t in np.arange(0, MAX_RT_DELTA, 1):
        t *= sign
        self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(t)))

      # Increase timer to update rt_torque_last
      self.safety.set_timer(RT_INTERVAL + 1)
      self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(sign * (MAX_RT_DELTA - 1))))
      self.assertTrue(self.safety.safety_tx_hook(self._torque_msg(sign * (MAX_RT_DELTA + 1))))


  def test_fwd_hook(self):
    # nothing allowed
    buss = list(range(0x0, 0x3))
    msgs = list(range(0x1, 0x800))

    for b in buss:
      for m in msgs:
        # assume len 8
        self.assertEqual(-1, self.safety.safety_fwd_hook(b, make_msg(b, m, 8)))


if __name__ == "__main__":
  unittest.main()
