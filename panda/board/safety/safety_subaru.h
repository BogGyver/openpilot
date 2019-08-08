const int SUBARU_MAX_STEER = 2047; // 1s
// real time torque limit to prevent controls spamming
// the real time limit is 1500/sec
const int SUBARU_MAX_RT_DELTA = 940;          // max delta torque allowed for real time checks
const uint32_t SUBARU_RT_INTERVAL = 250000;    // 250ms between real time checks
const int SUBARU_MAX_RATE_UP = 50;
const int SUBARU_MAX_RATE_DOWN = 70;
const int SUBARU_DRIVER_TORQUE_ALLOWANCE = 60;
const int SUBARU_DRIVER_TORQUE_FACTOR = 10;

int subaru_cruise_engaged_last = 0;
int subaru_rt_torque_last = 0;
int subaru_desired_torque_last = 0;
uint32_t subaru_ts_last = 0;
struct sample_t subaru_torque_driver;         // last few driver torques measured


static void subaru_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if ((addr == 0x119) && (bus == 0)){
    int torque_driver_new = ((GET_BYTES_04(to_push) >> 16) & 0x7FF);
    torque_driver_new = to_signed(torque_driver_new, 11);
    // update array of samples
    update_sample(&subaru_torque_driver, torque_driver_new);
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  if ((addr == 0x240) && (bus == 0)) {
    int cruise_engaged = GET_BYTE(to_push, 5) & 2;
    if (cruise_engaged && !subaru_cruise_engaged_last) {
      controls_allowed = 1;
    }
    if (!cruise_engaged) {
      controls_allowed = 0;
    }
    subaru_cruise_engaged_last = cruise_engaged;
  }
}

static int subaru_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);

  // steer cmd checks
  if (addr == 0x122) {
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x1FFF);
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
  if (bus_num == 0) {
    bus_fwd = 2;  // Camera CAN
  }
  if (bus_num == 2) {
    // 356 is LKAS for outback 2015
    // 356 is LKAS for Global Platform
    // 545 is ES_Distance
    // 802 is ES_LKAS
    int addr = GET_ADDR(to_fwd);
    int block_msg = (addr == 290) || (addr == 356) || (addr == 545) || (addr == 802);
    if (!block_msg) {
      bus_fwd = 0;  // Main CAN
    }
  }

  // fallback to do not forward
  return bus_fwd;
}

const safety_hooks subaru_hooks = {
  .init = nooutput_init,
  .rx = subaru_rx_hook,
  .tx = subaru_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = default_ign_hook,
  .fwd = subaru_fwd_hook,
};
