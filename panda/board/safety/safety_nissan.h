
const uint32_t NISSAN_RT_INTERVAL = 250000;    // 250ms between real time checks

const struct lookup_t NISSAN_LOOKUP_ANGLE_RATE_UP = {
  {2., 7., 17.},
  {5., .8, .15}};

const struct lookup_t NISSAN_LOOKUP_ANGLE_RATE_DOWN = {
  {2., 7., 17.},
  {5., 3.5, .5}};

const struct lookup_t NISSAN_LOOKUP_MAX_ANGLE = {
  {3.3, 12, 32},
  {540., 120., 23.}};

const int NISSAN_DEG_TO_CAN = 100;

const AddrBus NISSAN_TX_MSGS[] = {{0x169, 0}, {0x2b1, 0}, {0x4cc, 0}, {0x20b, 2}};

AddrCheckStruct nissan_rx_checks[] = {
  {.addr = {0x2}, .bus = 0, .expected_timestep = 10000U},
  {.addr = {0x29a}, .bus = 0, .expected_timestep = 20000U},
  {.addr = {0x1b6}, .bus = 1, .expected_timestep = 10000U},
};
const int NISSAN_RX_CHECK_LEN = sizeof(nissan_rx_checks) / sizeof(nissan_rx_checks[0]);

float nissan_speed = 0;
//int nissan_controls_allowed_last = 0;
uint32_t nissan_ts_angle_last = 0;
int nissan_cruise_engaged_last = 0;
int nissan_desired_angle_last = 0;

struct sample_t nissan_angle_meas;            // last 3 steer angles


static int nissan_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid = addr_safety_check(to_push, nissan_rx_checks, NISSAN_RX_CHECK_LEN,
                                 NULL, NULL, NULL);

  if (valid) {
    int bus = GET_BUS(to_push);
    int addr = GET_ADDR(to_push);

    if (bus == 0) {
      if (addr == 0x2) {
        // Current steering angle
        // Factor -0.1, little endian
        int angle_meas_new = (GET_BYTES_04(to_push) & 0xFFFF);
        // Need to multiply by 10 here as LKAS and Steering wheel are different base unit
        angle_meas_new = to_signed(angle_meas_new, 16) * 10;

        // update array of samples
        update_sample(&nissan_angle_meas, angle_meas_new);
      }

      if (addr == 0x29a) {
        // Get current speed
        // Factor 0.00555
        nissan_speed = ((GET_BYTE(to_push, 2) << 8) | (GET_BYTE(to_push, 3))) * 0.00555 / 3.6;
      }

      // exit controls on rising edge of gas press
      if (addr == 0x15c) {
        bool gas_pressed = ((GET_BYTE(to_push, 5) << 2) | ((GET_BYTE(to_push, 6) >> 6) & 0x3));
        if (gas_pressed && !gas_pressed_prev) {
          controls_allowed = 0;
        }
        gas_pressed_prev = gas_pressed;
      }

      // 0x169 is lkas cmd. If it is on bus 0, then relay is unexpectedly closed
      if ((safety_mode_cnt > RELAY_TRNS_TIMEOUT) && (addr == 0x169)) {
        relay_malfunction = true;
      }
    }

    if (bus == 1) {
      if (addr == 0x1b6) {
        int cruise_engaged = (GET_BYTE(to_push, 4) >> 6) & 1;
        if (cruise_engaged && !nissan_cruise_engaged_last) {
          controls_allowed = 1;
        }
        if (!cruise_engaged) {
          controls_allowed = 0;
        }
        nissan_cruise_engaged_last = cruise_engaged;
      }

      // exit controls on rising edge of brake press, or if speed > 0 and brake
      if (addr == 0x454) {
        bool brake_pressed = (GET_BYTE(to_push, 2) & 0x80) != 0;
        if (brake_pressed && (!brake_pressed_prev || (nissan_speed > 0.))) {
          controls_allowed = 0;
        }
        brake_pressed_prev = brake_pressed;
      }
    }
  }
  return valid;
}


static int nissan_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);
  bool violation = 0;

  if (!msg_allowed(addr, bus, NISSAN_TX_MSGS, sizeof(NISSAN_TX_MSGS) / sizeof(NISSAN_TX_MSGS[0]))) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // steer cmd checks
  if (addr == 0x169) {
    int desired_angle = ((GET_BYTE(to_send, 0) << 10) | (GET_BYTE(to_send, 1) << 2) | ((GET_BYTE(to_send, 2) >> 6) & 0x3));
    bool lka_active = (GET_BYTE(to_send, 6) >> 4) & 1;

    // offeset 1310 * NISSAN_DEG_TO_CAN
    desired_angle =  desired_angle - 131000;

    if (controls_allowed && lka_active) {
      // add 1 to not false trigger the violation
      float delta_angle_float;
      delta_angle_float = (interpolate(NISSAN_LOOKUP_ANGLE_RATE_UP, nissan_speed) * NISSAN_DEG_TO_CAN) + 1.;
      int delta_angle_up = (int)(delta_angle_float);
      delta_angle_float =  (interpolate(NISSAN_LOOKUP_ANGLE_RATE_DOWN, nissan_speed) * NISSAN_DEG_TO_CAN) + 1.;
      int delta_angle_down = (int)(delta_angle_float);
      int highest_desired_angle = nissan_desired_angle_last + ((nissan_desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = nissan_desired_angle_last - ((nissan_desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

      // Limit maximum steering angle at current speed
      int maximum_angle = ((int)interpolate(NISSAN_LOOKUP_MAX_ANGLE, nissan_speed));

      if (highest_desired_angle > (maximum_angle * NISSAN_DEG_TO_CAN)) {
        highest_desired_angle = (maximum_angle * NISSAN_DEG_TO_CAN);
      }
      if (lowest_desired_angle < (-maximum_angle * NISSAN_DEG_TO_CAN)) {
        lowest_desired_angle = (-maximum_angle * NISSAN_DEG_TO_CAN);
      }

      // check for violation;
      violation |= max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);

      //nissan_controls_allowed_last = controls_allowed;
    }
    nissan_desired_angle_last = desired_angle;

    // desired steer angle should be the same as steer angle measured when controls are off
    if ((!controls_allowed) &&
          ((desired_angle < (nissan_angle_meas.min - 1)) ||
          (desired_angle > (nissan_angle_meas.max + 1)))) {
      violation = 1;
    }

    // no lka_enabled bit if controls not allowed
    if (!controls_allowed && lka_active) {
      violation = 1;
    }
  }

  // acc button check, only allow cancel button to be sent
  if (addr == 0x20b) {
    // Violation of any button other than cancel is pressed
    violation |= ((GET_BYTE(to_send, 1) & 0x3d) > 0);
  }

  if (violation) {
    controls_allowed = 0;
    tx = 0;
  }

  return tx;
}


static int nissan_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  if (bus_num == 0) {
    bus_fwd = 2;  // ADAS
  }

  if (bus_num == 2) {
    // 0x169 is LKAS, 0x2b1 LKAS_HUD, 0x4cc LKAS_HUD_INFO_MSG
    int block_msg = ((addr == 0x169) || (addr == 0x2b1) || (addr == 0x4cc));
    if (!block_msg) {
      bus_fwd = 0;  // V-CAN
    }
  }

  if (relay_malfunction) {
    bus_fwd = -1;
  }

  // fallback to do not forward
  return bus_fwd;
}

const safety_hooks nissan_hooks = {
  .init = nooutput_init,
  .rx = nissan_rx_hook,
  .tx = nissan_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = nissan_fwd_hook,
  .addr_check = nissan_rx_checks,
  .addr_check_len = sizeof(nissan_rx_checks) / sizeof(nissan_rx_checks[0]),
};
