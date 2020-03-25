#!/usr/bin/env python3
import bz2
import os
import sys
import numbers

import dictdiffer
if "CI" in os.environ:
  tqdm = lambda x: x
else:
  from tqdm import tqdm

from tools.lib.logreader import LogReader

def save_log(dest, log_msgs):
  dat = b""
  for msg in tqdm(log_msgs):
    dat += msg.as_builder().to_bytes()
  dat = bz2.compress(dat)

  with open(dest, "wb") as f:
   f.write(dat)

def remove_ignored_fields(msg, ignore):
  msg = msg.as_builder()
  for key in ignore:
    attr = msg
    keys = key.split(".")
    if msg.which() not in key and len(keys) > 1:
      continue

    for k in keys[:-1]:
      try:
        attr = getattr(msg, k)
      except:
        break
    else:
      v = getattr(attr, keys[-1])
      if isinstance(v, bool):
        val = False
      elif isinstance(v, numbers.Number):
        val = 0
      else:
        raise NotImplementedError
      setattr(attr, keys[-1], val)
  return msg.as_reader()

def compare_logs(log1, log2, ignore_fields=[], ignore_msgs=[]):
  filter_msgs = lambda m: m.which() not in ignore_msgs
  log1, log2 = [list(filter(filter_msgs, log)) for log in (log1, log2)]
  assert len(log1) == len(log2), "logs are not same length: " + str(len(log1)) + " VS " + str(len(log2))

  diff = []
  for msg1, msg2 in tqdm(zip(log1, log2)):
    if msg1.which() != msg2.which():
      print(msg1, msg2)
      raise Exception("msgs not aligned between logs")

    msg1_bytes = remove_ignored_fields(msg1, ignore_fields).as_builder().to_bytes()
    msg2_bytes = remove_ignored_fields(msg2, ignore_fields).as_builder().to_bytes()

    if msg1_bytes != msg2_bytes:
      msg1_dict = msg1.to_dict(verbose=True)
      msg2_dict = msg2.to_dict(verbose=True)
      dd = dictdiffer.diff(msg1_dict, msg2_dict, ignore=ignore_fields, tolerance=0)
      diff.extend(dd)
  return diff

if __name__ == "__main__":
  log1 = list(LogReader(sys.argv[1]))
  log2 = list(LogReader(sys.argv[2]))
  print(compare_logs(log1, log2, sys.argv[3:]))
