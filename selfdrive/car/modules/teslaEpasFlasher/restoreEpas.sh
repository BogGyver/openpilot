#! /bin/sh
echo "Starting the restore process..."
cd /data/openpilot/selfdrive/car/modules/teslaEpasFlasher
PYTHONPATH=/data/openpilot 
./patch.py --restore
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " RESTORE PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo " Please hit Reboot to return to OP"
  echo "An error occurred during restore. Exiting..." >&2
  exit 1 
fi
echo "================================="
echo " RESTORE PROCESS COMPLETED"
echo "================================="
echo " Please hit Reboot to return to OP"