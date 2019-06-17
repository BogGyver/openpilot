import os
import time
from common.vin import is_vin_response_valid
from common.basedir import BASEDIR
from common.realtime import sec_since_boot
from common.fingerprints import eliminate_incompatible_cars, all_known_cars
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.swaglog import cloudlog
import selfdrive.messaging as messaging
from selfdrive.car.tesla.readconfig import CarSettings


def get_startup_alert(car_recognized, controller_available):
  alert = 'startup'
  if not car_recognized:
    alert = 'startupNoCar'
  elif car_recognized and not controller_available:
    alert = 'startupNoControl'
  return alert


def load_interfaces(brand_names):
  ret = {}
  for brand_name in brand_names:
    path = ('selfdrive.car.%s' % brand_name)
    CarInterface = __import__(path + '.interface', fromlist=['CarInterface']).CarInterface
    if os.path.exists(BASEDIR + '/' + path.replace('.', '/') + '/carcontroller.py'):
      CarController = __import__(path + '.carcontroller', fromlist=['CarController']).CarController
    else:
      CarController = None
    for model_name in brand_names[brand_name]:
      ret[model_name] = (CarInterface, CarController)
  return ret


def _get_interface_names():
  # read all the folders in selfdrive/car and return a dict where:
  # - keys are all the car names that which we have an interface for
  # - values are lists of spefic car models for a given car
  brand_names = {}
  for car_folder in [x[0] for x in os.walk(BASEDIR + '/selfdrive/car')]:
    try:
      brand_name = car_folder.split('/')[-1]
      model_names = __import__('selfdrive.car.%s.values' % brand_name, fromlist=['CAR']).CAR
      model_names = [getattr(model_names, c) for c in model_names.__dict__.keys() if not c.startswith("__")]
      brand_names[brand_name] = model_names
    except (ImportError, IOError):
      pass

  return brand_names


# imports from directory selfdrive/car/<name>/
interfaces = load_interfaces(_get_interface_names())


# BOUNTY: every added fingerprint in selfdrive/car/*/values.py is a $100 coupon code on shop.comma.ai
# **** for use live only ****
def fingerprint(logcan, sendcan):
  if os.getenv("SIMULATOR2") is not None:
    return ("simulator2", None, "")
  elif os.getenv("SIMULATOR") is not None:
    return ("simulator", None, "")

  finger = {}
  cloudlog.warning("waiting for fingerprint...")
  candidate_cars = all_known_cars()
  can_seen_ts = None
  can_seen = False

  # works on standard 11-bit addresses for diagnostic. Tested on Toyota and Subaru;
  # Honda uses the extended 29-bit addresses, and unfortunately only works from OBDII
  vin_query_msg = [[0x7df, 0, '\x02\x09\x02'.ljust(8, "\x00"), 0],
                   [0x7e0, 0, '\x30'.ljust(8, "\x00"), 0]]

  vin_cnts = [1, 2]  # number of messages to wait for at each iteration
  vin_step = 0
  vin_cnt = 0
  vin_responded = False
  vin_never_responded = True
  vin_dat = []
  vin = ""

  while 1:
    for a in messaging.drain_sock(logcan):
      for can in a.can:
        can_seen = True

        # have we got a VIN query response?
        if can.src == 0 and can.address == 0x7e8:
          vin_never_responded = False
          # basic sanity checks on ISO-TP response
          if is_vin_response_valid(can.dat, vin_step, vin_cnt):
            vin_dat += can.dat[2:] if vin_step == 0 else can.dat[1:]
            vin_cnt += 1
            if vin_cnt == vin_cnts[vin_step]:
              vin_responded = True
              vin_step += 1

        # ignore everything not on bus 0 and with more than 11 bits,
        # which are ussually sporadic and hard to include in fingerprints.
        # also exclude VIN query response on 0x7e8
        if can.src == 0 and can.address < 0x7df and can.address != 0x7e8:
          finger[can.address] = len(can.dat)
          candidate_cars = eliminate_incompatible_cars(can, candidate_cars)
          print can.address, len(can.dat), candidate_cars

    if can_seen_ts is None and can_seen:
      can_seen_ts = sec_since_boot()          # start time
    ts = sec_since_boot()
    # if we only have one car choice and the time_fingerprint since we got our first
    # message has elapsed, exit. Toyota needs higher time_fingerprint, since DSU does not
    # broadcast immediately
    if len(candidate_cars) == 1 and can_seen_ts is not None:
      time_fingerprint = 1.0 if ("TOYOTA" in candidate_cars[0] or "LEXUS" in candidate_cars[0]) else 0.1
      if (ts - can_seen_ts) > time_fingerprint:
        break

    # bail if no cars left or we've been waiting for more than 2s since can_seen
    elif len(candidate_cars) == 0 or (can_seen_ts is not None and (ts - can_seen_ts) > 2.):
      if CarSettings().forceFingerprintTesla:
        print "Fingerprinting Failed: Returning Tesla (based on branch)"
        return "TESLA MODEL S", finger, "TESLAFAKEVIN"
      else:
        return None, finger, ""

    # keep sending VIN qury if ECU isn't responsing.
    # sendcan is probably not ready due to the zmq slow joiner syndrome
    if can_seen and (vin_never_responded or (vin_responded and vin_step < len(vin_cnts))):
      sendcan.send(can_list_to_can_capnp([vin_query_msg[vin_step]], msgtype='sendcan'))
      vin_responded = False
      vin_cnt = 0

    time.sleep(0.01)

  # only report vin if procedure is finished
  if vin_step == len(vin_cnts) and vin_cnt == vin_cnts[-1]:
    vin = "".join(vin_dat[3:])

  cloudlog.warning("fingerprinted %s", candidate_cars[0])
  cloudlog.warning("VIN %s", vin)
  return candidate_cars[0], finger, vin


def get_car(logcan, sendcan):
  if CarSettings().forceFingerprintTesla:
    candidate="TESLA MODEL S"
    fingerprints=""
    vin="TESLAFORCED"
  else:
    candidate, fingerprints, vin = fingerprint(logcan, sendcan)

  if candidate is None:
    cloudlog.warning("car doesn't match any fingerprints: %r", fingerprints)
    candidate = "mock"

  CarInterface, CarController = interfaces[candidate]
  params = CarInterface.get_params(candidate, fingerprints, vin)

  return CarInterface(params, CarController), params
