#!/usr/bin/env python3
import time
import argparse
from panda import Panda
from panda.board.pedal.canhandle import CanHandle
from selfdrive.car.modules.CFG_module import load_bool_param


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Flash pedal over can')
  parser.add_argument('--recover', action='store_true')
  parser.add_argument("fn", type=str, nargs='?', help="flash file")
  args = parser.parse_args()

  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  pedalcan = 2
  if load_bool_param("TinklaPedalCanZero", False):
      pedalcan = 0

  while 1:
    if len(p.can_recv()) == 0:
      break

  if args.recover:
    p.can_send(0x200, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x02", pedalcan)
    p.can_send(0x551, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x02", pedalcan)
    exit(0)
  else:
    p.can_send(0x200, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x0a", pedalcan)
    p.can_send(0x551, b"\xce\xfa\xad\xde\x1e\x0b\xb0\x02", pedalcan)

  if args.fn:
    time.sleep(0.1)
    print("flashing", args.fn)
    code = open(args.fn, "rb").read()
    Panda.flash_static(CanHandle(p, pedalcan), code)

  print("can flash done")
