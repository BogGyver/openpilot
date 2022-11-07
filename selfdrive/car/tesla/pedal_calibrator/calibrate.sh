#! /bin/sh
echo "================================="
echo "CALIBRATION PROCESS STARTED"
echo "================================="
echo "NOTE: KEEP BRAKE PEDAL PRESSED UNTIL FLASH PROCESS IS COMPLETE"
echo "  "
echo "Starting the calibration process..."
echo "  "
cd /data/openpilot/selfdrive/car/tesla/pedal_calibrator
PYTHONPATH=/data/openpilot 
./calibratePedal.py 
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " CALIBRATION PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo " Please hit Reboot to return to OP"
  echo "An error occurred during calibration. Exiting..." >&2
  exit 1
fi
echo "================================="
echo " CALIBRATION PROCESS COMPLETED"
echo "================================="
echo " Please hit Reboot to return to OP"
exit 0
