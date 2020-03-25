#!/usr/bin/env python3
import argparse
import numpy as np
from collections import defaultdict, deque
from common.realtime import sec_since_boot
import cereal.messaging as messaging


if __name__ == "__main__":
  context = messaging.Context()
  poller = messaging.Poller()

  parser = argparse.ArgumentParser()
  parser.add_argument("socket", type=str, nargs='*', help="socket name")
  args = parser.parse_args()

  socket_names = args.socket
  sockets = {}

  rcv_times = defaultdict(lambda: deque(maxlen=100))

  t = sec_since_boot()
  for name in socket_names:
    sock = messaging.sub_sock(name, poller=poller)
    sockets[sock] = name

  prev_print = t
  while True:
    for socket in poller.poll(100):
      msg = messaging.recv_one(socket)
      name = msg.which()

      t = sec_since_boot()
      rcv_times[name].append(msg.logMonoTime / 1e9)

    if t - prev_print > 1:
      print()
      for name in socket_names:
        dts = np.diff(rcv_times[name])
        mean = np.mean(dts)
        print("%s: Freq %.2f Hz, Min %.2f%%, Max %.2f%%" % (name, 1.0 / mean, np.min(dts) / mean * 100, np.max(dts) / mean * 100))

      prev_print = t
