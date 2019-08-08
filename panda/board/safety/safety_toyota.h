// global torque limit
const int TOYOTA_MAX_TORQUE = 1500;       // max torque cmd allowed ever

// rate based torque limit + stay within actually applied
// packet is sent at 100hz, so this limit is 1000/sec
const int TOYOTA_MAX_RATE_UP = 10;        // ramp up slow
const int TOYOTA_MAX_RATE_DOWN = 25;      // ramp down fast
const int TOYOTA_MAX_TORQUE_ERROR = 350;  // max torque cmd in excess of torque motor

// real time torque limit to prevent controls spamming
// the real time limit is 1500/sec
const int TOYOTA_MAX_RT_DELTA = 375;      // max delta torque allowed for real time checks
const uint32_t TOYOTA_RT_INTERVAL = 250000;    // 250ms between real time checks

// longitudinal limits
const int TOYOTA_MAX_ACCEL = 1500;        // 1.5 m/s2
const int TOYOTA_MIN_ACCEL = -3000;       // 3.0 m/s2

const int TOYOTA_GAS_INTERCEPTOR_THRESHOLD = 475;  // ratio between offset and gain from dbc file

// global actuation limit states
int toyota_dbc_eps_torque_factor = 100;   // conversion factor for STEER_TORQUE_EPS in %: see dbc file

// states
int toyota_giraffe_switch_1 = 0;          // is giraffe switch 1 high?
int toyota_camera_forwarded = 0;          // should we forward the camera bus?
int toyota_desired_torque_last = 0;       // last desired steer torque
int toyota_rt_torque_last = 0;            // last desired torque for real time check
uint32_t toyota_ts_last = 0;
int toyota_cruise_engaged_last = 0;       // cruise state
int toyota_gas_prev = 0;
struct sample_t toyota_torque_meas;       // last 3 motor torques produced by the eps


static void toyota_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  // get eps motor torque (0.66 factor in dbc)
  if (addr == 0x260) {
    int torque_meas_new = (GET_BYTE(to_push, 5) << 8) | GET_BYTE(to_push, 6);
    torque_meas_new = to_signed(torque_meas_new, 16);

    // scale by dbc_factor
    torque_meas_new = (torque_meas_new * toyota_dbc_eps_torque_factor) / 100;

    // update array of sample
    update_sample(&toyota_torque_meas, torque_meas_new);

    // increase torque_meas by 1 to be conservative on rounding
    toyota_torque_meas.min--;
    toyota_torque_meas.max++;
  }

  // enter controls on rising edge of ACC, exit controls on ACC off
  if (addr == 0x1D2) {
    // 5th bit is CRUISE_ACTIVE
    int cruise_engaged = GET_BYTE(to_push, 0) & 0x20;
    if (!cruise_engaged) {
      controls_allowed = 0;
    }
    if (cruise_engaged && !toyota_cruise_engaged_last) {
      controls_allowed = 1;
    }
    toyota_cruise_engaged_last = cruise_engaged;
  }

  // exit controls on rising edge of interceptor gas press
  if (addr == 0x201) {
    gas_interceptor_detected = 1;
    int gas_interceptor = GET_INTERCEPTOR(to_push);
    if ((gas_interceptor > TOYOTA_GAS_INTERCEPTOR_THRESHOLD) &&
        (gas_interceptor_prev <= TOYOTA_GAS_INTERCEPTOR_THRESHOLD) &&
        long_controls_allowed) {
      controls_allowed = 0;
    }
    gas_interceptor_prev = gas_interceptor;
  }

  // exit controls on rising edge of gas press
  if (addr == 0x2C1) {
    int gas = GET_BYTE(to_push, 6) & 0xFF;
    if ((gas > 0) && (toyota_gas_prev == 0) && !gas_interceptor_detected && long_controls_allowed) {
      controls_allowed = 0;
    }
    toyota_gas_prev = gas;
  }

  // msgs are only on bus 2 if panda is connected to frc
  if (bus == 2) {
    toyota_camera_forwarded = 1;
  }

  // 0x2E4 is lkas cmd. If it is on bus 0, then giraffe switch 1 is high
  if ((addr == 0x2E4) && (bus == 0)) {
    toyota_giraffe_switch_1 = 1;
  }
}

static int toyota_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  // Check if msg is sent on BUS 0
  if (bus == 0) {

    // no IPAS in non IPAS mode
    if ((addr == 0x266) || (addr == 0x167)) {
      tx = 0;
    }

    // GAS PEDAL: safety check
    if (addr == 0x200) {
      if (!controls_allowed || !long_controls_allowed) {
        if (GET_BYTE(to_send, 0) || GET_BYTE(to_send, 1)) {
          tx = 0;
        }
      }
    }

    // ACCEL: safety check on byte 1-2
    if (addr == 0x343) {
      int desired_accel = (GET_BYTE(to_send, 0) << 8) | GET_BYTE(to_send, 1);
      desired_accel = to_signed(desired_accel, 16);
      if (!controls_allowed || !long_controls_allowed) {
        if (desired_accel != 0) {
          tx = 0;
        }
      }
      bool violation = max_limit_check(desired_accel, TOYOTA_MAX_ACCEL, TOYOTA_MIN_ACCEL);
      if (violation) {
        tx = 0;
      }
    }

    // STEER: safety check on bytes 2-3
    if (addr == 0x2E4) {
      int desired_torque = (GET_BYTE(to_send, 1) << 8) | GET_BYTE(to_send, 2);
      desired_torque = to_signed(desired_torque, 16);
      bool violation = 0;

      uint32_t ts = TIM2->CNT;

      if (controls_allowed) {

        // *** global torque limit check ***
        violation |= max_limit_check(desired_torque, TOYOTA_MAX_TORQUE, -TOYOTA_MAX_TORQUE);

        // *** torque rate limit check ***
        violation |= dist_to_meas_check(desired_torque, toyota_desired_torque_last,
          &toyota_torque_meas, TOYOTA_MAX_RATE_UP, TOYOTA_MAX_RATE_DOWN, TOYOTA_MAX_TORQUE_ERROR);

        // used next time
        toyota_desired_torque_last = desired_torque;

        // *** torque real time rate limit check ***
        violation |= rt_rate_limit_check(desired_torque, toyota_rt_torque_last, TOYOTA_MAX_RT_DELTA);

        // every RT_INTERVAL set the new limits
        uint32_t ts_elapsed = get_ts_elapsed(ts, toyota_ts_last);
        if (ts_elapsed > TOYOTA_RT_INTERVAL) {
          toyota_rt_torque_last = desired_torque;
          toyota_ts_last = ts;
        }
      }

      // no torque if controls is not allowed
      if (!controls_allowed && (desired_torque != 0)) {
        violation = 1;
      }

      // reset to 0 if either controls is not allowed or there's a violation
      if (violation || !controls_allowed) {
        toyota_desired_torque_last = 0;
        toyota_rt_torque_last = 0;
        toyota_ts_last = ts;
      }

      if (violation) {
        tx = 0;
      }
    }
  }

  // 1 allows the message through
  return tx;
}

static void toyota_init(int16_t param) {
  controls_allowed = 0;
  toyota_giraffe_switch_1 = 0;
  toyota_camera_forwarded = 0;
  toyota_dbc_eps_torque_factor = param;
}

static int toyota_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  if (toyota_camera_forwarded && !toyota_giraffe_switch_1) {
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    if (bus_num == 2) {
      int addr = GET_ADDR(to_fwd);
      // block stock lkas messages and stock acc messages (if OP is doing ACC)
      // in TSS2, 0.191 is LTA which we need to block to avoid controls collision
      int is_lkas_msg = ((addr == 0x2E4) || (addr == 0x412) || (addr == 0x191));
      // in TSS2 the camera does ACC as well, so filter 0x343
      int is_acc_msg = (addr == 0x343);
      int block_msg = is_lkas_msg || (is_acc_msg && long_controls_allowed);
      if (!block_msg) {
        bus_fwd = 0;
      }
    }
  }
  return bus_fwd;
}

const safety_hooks toyota_hooks = {
  .init = toyota_init,
  .rx = toyota_rx_hook,
  .tx = toyota_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = default_ign_hook,
  .fwd = toyota_fwd_hook,
};
