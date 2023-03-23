#!/usr/bin/env python3
import os
import sys
from common.basedir import BASEDIR

# messages reserved for CAN based ignition (see can_ignition_hook function in panda/board/drivers/can)
# (addr, len)
CAN_IGNITION_MSGS = {
  'gm': [(0x1F1, 8), (0x160, 5)],
  #'tesla' : [(0x348, 8)],
}

def _get_fingerprints():
  # read all the folders in selfdrive/car and return a dict where:
  # - keys are all the car names that which we have a fingerprint dict for
  # - values are dicts of fingeprints for each trim
  fingerprints = {}
  for car_folder in [x[0] for x in os.walk(BASEDIR + '/selfdrive/car')]:
    car_name = car_folder.split('/')[-1]
    try:
      fingerprints[car_name] = __import__('selfdrive.car.%s.values' % car_name, fromlist=['FINGERPRINTS']).FINGERPRINTS
    except (ImportError, IOError, AttributeError):
      pass

  return fingerprints


def check_fingerprint_consistency(f1, f2):
  # return false if it finds a fingerprint fully included in another
  # max message worth checking is 1800, as above that they usually come too infrequently and not
  # usable for fingerprinting

  max_msg = 1800

  is_f1_in_f2 = True
  for k in f1:
    if (k not in f2 or f1[k] != f2[k]) and k < max_msg:
       is_f1_in_f2 = False

  is_f2_in_f1 = True
  for k in f2:
    if (k not in f1 or f2[k] != f1[k]) and k < max_msg:
       is_f2_in_f1 = False

  return not is_f1_in_f2 and not is_f2_in_f1


def check_can_ignition_conflicts(fingerprints, brands):
  # loops through all the fingerprints and exits if CAN ignition dedicated messages
  # are found in unexpected fingerprints

  for brand_can, msgs_can in CAN_IGNITION_MSGS.items():
    for i, f in enumerate(fingerprints):
      for msg_can in msgs_can:
        if brand_can != brands[i] and msg_can[0] in f and msg_can[1] == f[msg_can[0]]:
          print("CAN ignition dedicated msg %d with len %d found in %s fingerprints!" % (msg_can[0], msg_can[1], brands[i]))
          print("TEST FAILED")
          sys.exit(1)


fingerprints = _get_fingerprints()

fingerprints_flat = []
car_names = []
brand_names = []
for brand in fingerprints:
  for car in fingerprints[brand]:

    fingerprints_flat += fingerprints[brand][car]
    for i in range(len(fingerprints[brand][car])):
      car_names.append(car)
      brand_names.append(brand)

if len(sys.argv) != 2 or not sys.argv[1] in ["-i","-c"]:
  print("   ****   Fingerprint Tester   ****")
  print(" ")
  print("use 'python",sys.argv[0],"-i' to see which car matches your fingerprint.")
  print(" ")
  print("use 'python",sys.argv[0],"-c' to see what message IDs makes it fail against a certain car.")
  exit(0)

count = 0
idx2 = "YOUR_CAR"
f2string = input("Please provide your fingerprint:")
f2l = f2string.split(",")
f2 = {}
for a in f2l:
    b,c = a.split(":")
    f2[int(b)]=int(c)
#print("You entered: [",f2,"]")
if sys.argv[1] == "-i":
  for idx1, f1 in enumerate(fingerprints_flat):
      if not check_fingerprint_consistency(f1, f2):
        print("{0} matches the fingerprint for {1}".format(idx2, car_names[idx1]))
        print("")
        count = count + 1


if sys.argv[1] == "-c":
  fingerprints_flat = []
  car_names = []
  brands = {}
  brand_names = []
  bid = 0

  print("Please select the brand to check against")
  for brand in fingerprints:
    if brand not in brand_names:
      brand_names.append(brand)
      bid = bid + 1
      brands[bid] = brand
      print("{} : {}".format(bid,brand))
  brandmod = input("Brand # to check against:")
  brandt = brands[int(brandmod)]

  cars = {}
  cid = 0
  for car in fingerprints[brandt]:
    fingerprints_flat += fingerprints[brandt][car]
    cid = cid + 1
    cars[cid]=car
    for i in range(len(fingerprints[brandt][car])):
      car_names.append(car)
    print("{} : {}".format(cid,car))

  carmod = input("Car # to check against:")
  car = cars[int(carmod)]
  included_in = False
  for idx1, f1 in enumerate(fingerprints_flat):
    if car == car_names[idx1]:
      print (car,"(",idx1,")")
      all_in = True
      for k in f2:
        if (k not in f1):
          print("{}: {} not in fingerprint for {} {}".format(k,f2[k],brandt,car))
          all_in = False
        if (k in f1) and (f2[k] != f1[k]):
          print("{}: {} does not match the size ({}) in fingerprint for {} {}".format(k,f2[k],f1[k],brandt,car))
          all_in = False
      
      if not all_in:
        print("Not fully included!")
      else:
        print("Fully included!")
        included_in = True
  if included_in:
    print("PASS: At least one of the fingerprints for",car,"contains your fingerprint!")
  else:
    print("FAIL: None of the fingerprints for",car,"contains your fingerprint!")