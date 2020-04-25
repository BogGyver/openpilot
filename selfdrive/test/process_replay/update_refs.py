#!/usr/bin/env python3
import os
import sys

#from selfdrive.test.openpilotci_upload import upload_file
from selfdrive.test.process_replay.compare_logs import save_log
from selfdrive.test.process_replay.process_replay import replay_process, CONFIGS
from selfdrive.test.process_replay.test_processes import segments, get_segment
from selfdrive.version import get_git_commit
from tools.lib.logreader import LogReader

if __name__ == "__main__":

  #no_upload = "--no-upload" in sys.argv

  process_replay_dir = os.path.dirname(os.path.abspath(__file__))
  ref_files_dir = os.path.join(process_replay_dir, "ref_files")
  ref_commit_fn = os.path.join(process_replay_dir, "ref_commit")

  #first delete all old files in directory
  filelist = [ f for f in os.listdir(ref_files_dir) if f.endswith(".bz2") ]
  for f in filelist:
    os.remove(os.path.join(ref_files_dir, f))

  #now just follow standard Comma process
  ref_commit = get_git_commit()
  with open(ref_commit_fn, "w") as f:
    f.write(ref_commit)

  for car_brand, segment in segments:
    rlog_fn = get_segment(segment)

    if rlog_fn is None:
      print("failed to get segment %s" % segment)
      sys.exit(1)

    lr = LogReader(rlog_fn)

    for cfg in CONFIGS:
      log_msgs = replay_process(cfg, lr)
      log_fn = os.path.join(ref_files_dir, "%s_%s_%s.bz2" % (segment, cfg.proc_name, ref_commit))
      save_log(log_fn, log_msgs)

      #if not no_upload:
      #  upload_file(log_fn, os.path.basename(log_fn))
      #  os.remove(log_fn)
    os.remove(rlog_fn)

  print("done")
