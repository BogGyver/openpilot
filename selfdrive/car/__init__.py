# functions common among cars
from common.numpy_fast import clip

# kg of standard extra cargo to count for drive, gas, etc...
STD_CARGO_KG = 136.

def gen_empty_fingerprint():
  return {i: {} for i in range(0, 4)}

# FIXME: hardcoding honda civic 2016 touring params so they can be used to
# scale unknown params for other cars
class CivicParams:
  MASS = 1326. + STD_CARGO_KG
  WHEELBASE = 2.70
  CENTER_TO_FRONT = WHEELBASE * 0.4
  CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT
  ROTATIONAL_INERTIA = 2500
  TIRE_STIFFNESS_FRONT = 192150
  TIRE_STIFFNESS_REAR = 202500

# TODO: get actual value, for now starting with reasonable value for
# civic and scaling by mass and wheelbase
def scale_rot_inertia(mass, wheelbase):
  return CivicParams.ROTATIONAL_INERTIA * mass * wheelbase ** 2 / (CivicParams.MASS * CivicParams.WHEELBASE ** 2)

# TODO: start from empirically derived lateral slip stiffness for the civic and scale by
# mass and CG position, so all cars will have approximately similar dyn behaviors
def scale_tire_stiffness(mass, wheelbase, center_to_front, tire_stiffness_factor=1.0):
  center_to_rear = wheelbase - center_to_front
  tire_stiffness_front = (CivicParams.TIRE_STIFFNESS_FRONT * tire_stiffness_factor) * mass / CivicParams.MASS * \
                         (center_to_rear / wheelbase) / (CivicParams.CENTER_TO_REAR / CivicParams.WHEELBASE)

  tire_stiffness_rear = (CivicParams.TIRE_STIFFNESS_REAR * tire_stiffness_factor) * mass / CivicParams.MASS * \
                        (center_to_front / wheelbase) / (CivicParams.CENTER_TO_FRONT / CivicParams.WHEELBASE)

  return tire_stiffness_front, tire_stiffness_rear

def dbc_dict(pt_dbc, radar_dbc, chassis_dbc=None):
  return {'pt': pt_dbc, 'radar': radar_dbc, 'chassis': chassis_dbc}


def apply_std_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):

  # limits due to driver torque
  driver_max_torque = LIMITS.STEER_MAX + (LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
  driver_min_torque = -LIMITS.STEER_MAX + (-LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
  max_steer_allowed = max(min(LIMITS.STEER_MAX, driver_max_torque), 0)
  min_steer_allowed = min(max(-LIMITS.STEER_MAX, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                                    apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                                    min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))


def apply_toyota_steer_torque_limits(apply_torque, apply_torque_last, motor_torque, LIMITS):
  # limits due to comparison of commanded torque VS motor reported torque
  max_lim = min(max(motor_torque + LIMITS.STEER_ERROR_MAX, LIMITS.STEER_ERROR_MAX), LIMITS.STEER_MAX)
  min_lim = max(min(motor_torque - LIMITS.STEER_ERROR_MAX, -LIMITS.STEER_ERROR_MAX), -LIMITS.STEER_MAX)

  apply_torque = clip(apply_torque, min_lim, max_lim)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque,
                        max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque,
                        apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))


def crc8_pedal(data):
  crc = 0xFF    # standard init value
  poly = 0xD5   # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size-1, -1, -1):
    crc ^= data[i]
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc


def create_gas_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  dat = packer.make_can_msg("GAS_COMMAND", 0, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", 0, values)


def is_ecu_disconnected(fingerprint, fingerprint_list, ecu_fingerprint, car, ecu):
  # check if a stock ecu is disconnected by looking for specific CAN msgs in the fingerprint
  # return True if the reference car fingerprint contains the ecu fingerprint msg and
  # fingerprint does not contains messages normally sent by a given ecu
  ecu_in_car = False
  for car_finger in fingerprint_list[car]:
    if any(msg in car_finger for msg in ecu_fingerprint[ecu]):
      ecu_in_car = True

  return ecu_in_car and not any(msg in fingerprint for msg in ecu_fingerprint[ecu])


def make_can_msg(addr, dat, bus):
  return [addr, 0, dat, bus]

