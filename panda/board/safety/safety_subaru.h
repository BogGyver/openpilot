const int SUBARU_MAX_STEER = 2047; // 1s
// real time torque limit to prevent controls spamming
// the real time limit is 1500/sec
const int SUBARU_MAX_RT_DELTA = 940;          // max delta torque allowed for real time checks
const uint32_t SUBARU_RT_INTERVAL = 250000;    // 250ms between real time checks
const int SUBARU_MAX_RATE_UP = 50;
const int SUBARU_MAX_RATE_DOWN = 70;
const int SUBARU_DRIVER_TORQUE_ALLOWANCE = 60;
const int SUBARU_DRIVER_TORQUE_FACTOR = 10;
const int SUBARU_STANDSTILL_THRSLD = 20;  // about 1kph

const AddrBus SUBARU_TX_MSGS[] = {{0x122, 0}, {0x221, 0}, {0x322, 0}};
const AddrBus SUBARU_L_TX_MSGS[] = {{0x164, 0}, {0x221, 0}, {0x322, 0}};
const int SUBARU_TX_MSGS_LEN = sizeof(SUBARU_TX_MSGS) / sizeof(SUBARU_TX_MSGS[0]);
const int SUBARU_L_TX_MSGS_LEN = sizeof(SUBARU_L_TX_MSGS) / sizeof(SUBARU_L_TX_MSGS[0]);

AddrCheckStruct subaru_rx_checks[] = {
  {.addr = { 0x40}, .bus = 0, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U},
  {.addr = {0x119}, .bus = 0, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U},
  {.addr = {0x139}, .bus = 0, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U},
  {.addr = {0x13a}, .bus = 0, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U},
  {.addr = {0x240}, .bus = 0, .check_checksum = true, .max_counter = 15U, .expected_timestep = 50000U},
};
// TODO: do checksum and counter checks after adding the signals to the outback dbc file
AddrCheckStruct subaru_l_rx_checks[] = {
  {.addr = {0x140}, .bus = 0, .expected_timestep = 10000U},
  {.addr = {0x371}, .bus = 0, .expected_timestep = 20000U},
  {.addr = {0x144}, .bus = 0, .expected_timestep = 50000U},
};
const int SUBARU_RX_CHECK_LEN = sizeof(subaru_rx_checks) / sizeof(subaru_rx_checks[0]);
const int SUBARU_L_RX_CHECK_LEN = sizeof(subaru_l_rx_checks) / sizeof(subaru_l_rx_checks[0]);

int subaru_cruise_engaged_last = 0;
int subaru_rt_torque_last = 0;
int subaru_desired_torque_last = 0;
int subaru_speed = 0;
uint32_t subaru_ts_last = 0;
bool subaru_global = false;
struct sample_t subaru_torque_driver;         // last few driver torques measured

static uint8_t subaru_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  return (uint8_t)GET_BYTE(to_push, 0);
}

static uint8_t subaru_get_counter(CAN_FIFOMailBox_TypeDef *to_push) {
  return (uint8_t)(GET_BYTE(to_push, 1) & 0xF);
}

static uint8_t subaru_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  uint8_t checksum = (uint8_t)(addr) + (uint8_t)((unsigned int)(addr) >> 8U);
  for (int i = 1; i < len; i++) {
    checksum += (uint8_t)GET_BYTE(to_push, i);
  }
  return checksum;
}

static int subaru_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid = false;
  if (subaru_global) {
    valid = addr_safety_check(to_push, subaru_rx_checks, SUBARU_RX_CHECK_LEN,
                              subaru_get_checksum, subaru_compute_checksum, subaru_get_counter);
  } else {
    valid = addr_safety_check(to_push, subaru_l_rx_checks, SUBARU_L_RX_CHECK_LEN,
                              NULL, NULL, NULL);
  }

  if (valid && (GET_BUS(to_push) == 0)) {
    int addr = GET_ADDR(to_push);
    if (((addr == 0x119) && subaru_global) ||
        ((addr == 0x371) && !subaru_global)) {
      int torque_driver_new;
      if (subaru_global) {
        torque_driver_new = ((GET_BYTES_04(to_push) >> 16) & 0x7FF);
      } else {
        torque_driver_new = (GET_BYTE(to_push, 3) >> 5) + (GET_BYTE(to_push, 4) << 3);
      }
      torque_driver_new = to_signed(torque_driver_new, 11);
      update_sample(&subaru_torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (((addr == 0x240) && subaru_global) ||
        ((addr == 0x144) && !subaru_global)) {
      int bit_shift = subaru_global ? 9 : 17;
      int cruise_engaged = ((GET_BYTES_48(to_push) >> bit_shift) & 1);
      if (cruise_engaged && !subaru_cruise_engaged_last) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      subaru_cruise_engaged_last = cruise_engaged;
    }

    // sample subaru wheel speed, averaging opposite corners
    if ((addr == 0x13a) && subaru_global) {
      subaru_speed = (GET_BYTES_04(to_push) >> 12) & 0x1FFF;  // FR
      subaru_speed += (GET_BYTES_48(to_push) >> 6) & 0x1FFF;  // RL
      subaru_speed /= 2;
    }

    // exit controls on rising edge of brake press (TODO: missing check for unsupported legacy models)
    if ((addr == 0x139) && subaru_global) {
      bool brake_pressed = (GET_BYTES_48(to_push) & 0xFFF0) > 0;
      if (brake_pressed && (!brake_pressed_prev || (subaru_speed > SUBARU_STANDSTILL_THRSLD))) {
        controls_allowed = 0;
      }
      brake_pressed_prev = brake_pressed;
    }

    // exit controls on rising edge of gas press
    if (((addr == 0x40) && subaru_global) ||
        ((addr == 0x140) && !subaru_global)) {
      int byte = subaru_global ? 4 : 0;
      bool gas_pressed = GET_BYTE(to_push, byte) != 0;
      if (gas_pressed && !gas_pressed_prev) {
        controls_allowed = 0;
      }
      gas_pressed_prev = gas_pressed;
    }

    if ((safety_mode_cnt > RELAY_TRNS_TIMEOUT) &&
        (((addr == 0x122) && subaru_global) || ((addr == 0x164) && !subaru_global))) {
      relay_malfunction = true;
    }
  }
  return valid;
}

static int subaru_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if ((!msg_allowed(addr, bus, SUBARU_TX_MSGS, SUBARU_TX_MSGS_LEN) && subaru_global) ||
      (!msg_allowed(addr, bus, SUBARU_L_TX_MSGS, SUBARU_L_TX_MSGS_LEN) && !subaru_global)) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // steer cmd checks
  if (((addr == 0x122) && subaru_global) ||
      ((addr == 0x164) && !subaru_global)) {
    int bit_shift = subaru_global ? 16 : 8;
    int desired_torque = ((GET_BYTES_04(to_send) >> bit_shift) & 0x1FFF);
    bool violation = 0;
    uint32_t ts = TIM2->CNT;
    desired_torque = to_signed(desired_torque, 13);

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, SUBARU_MAX_STEER, -SUBARU_MAX_STEER);

      // *** torque rate limit check ***
      int desired_torque_last = subaru_desired_torque_last;
      violation |= driver_limit_check(desired_torque, desired_torque_last, &subaru_torque_driver,
        SUBARU_MAX_STEER, SUBARU_MAX_RATE_UP, SUBARU_MAX_RATE_DOWN,
        SUBARU_DRIVER_TORQUE_ALLOWANCE, SUBARU_DRIVER_TORQUE_FACTOR);

      // used next time
      subaru_desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, subaru_rt_torque_last, SUBARU_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, subaru_ts_last);
      if (ts_elapsed > SUBARU_RT_INTERVAL) {
        subaru_rt_torque_last = desired_torque;
        subaru_ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      subaru_desired_torque_last = 0;
      subaru_rt_torque_last = 0;
      subaru_ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }

  }
  return tx;
}

static int subaru_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;

  if (!relay_malfunction) {
    if (bus_num == 0) {
      bus_fwd = 2;  // Camera CAN
    }
    if (bus_num == 2) {
      // 290 is LKAS for Global Platform
      // 356 is LKAS for outback 2015
      // 545 is ES_Distance
      // 802 is ES_LKAS
      int addr = GET_ADDR(to_fwd);
      int block_msg = ((addr == 0x122) && subaru_global) ||
                      ((addr == 0x164) && !subaru_global) ||
                      (addr == 0x221) || (addr == 0x322);
      if (!block_msg) {
        bus_fwd = 0;  // Main CAN
      }
    }
  }
  // fallback to do not forward
  return bus_fwd;
}

static void subaru_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction = false;
  subaru_global = true;
}

static void subaru_legacy_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction = false;
  subaru_global = false;
}

const safety_hooks subaru_hooks = {
  .init = subaru_init,
  .rx = subaru_rx_hook,
  .tx = subaru_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = subaru_fwd_hook,
  .addr_check = subaru_rx_checks,
  .addr_check_len = sizeof(subaru_rx_checks) / sizeof(subaru_rx_checks[0]),
};

const safety_hooks subaru_legacy_hooks = {
  .init = subaru_legacy_init,
  .rx = subaru_rx_hook,
  .tx = subaru_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = subaru_fwd_hook,
  .addr_check = subaru_l_rx_checks,
  .addr_check_len = sizeof(subaru_l_rx_checks) / sizeof(subaru_l_rx_checks[0]),
};
