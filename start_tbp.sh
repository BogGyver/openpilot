#!/bin/bash
export PYTHONPATH=/data/openpilot
cd /data/openpilot/selfdrive
PASSIVE=0 NOSENSOR=1 WEBCAM=1 ./manager.py
