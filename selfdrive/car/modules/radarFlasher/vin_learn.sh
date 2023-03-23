#! /bin/sh
echo "================================="
echo "VIN LEARN PROCESS STARTED"
echo "================================="
echo "NOTE: KEEP BRAKE PEDAL PRESSED UNTIL PROCESS IS COMPLETE"
echo "  "
echo "Starting the VIN learn process..."
echo "  "
cd /data/openpilot/selfdrive/car/modules/radarFlasher
PYTHONPATH=/data/openpilot 
./patch_radar.py --vin-learn
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " VIN LEARN PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo " Please hit Reboot to return to OP"
  echo "An error occurred during VIN learn. Exiting..." >&2
  exit 1
fi
echo "================================="
echo " VIN LEARN PROCESS COMPLETED"
echo "================================="
echo " Please hit Reboot to return to OP"
exit 0
