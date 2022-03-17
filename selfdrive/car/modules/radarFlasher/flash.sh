#! /bin/sh
echo "================================="
echo "FLASH PROCESS STARTED"
echo "================================="
echo "NOTE: KEEP BRAKE PEDAL PRESSED UNTIL FLASH PROCESS IS COMPLETE"
echo "  "
echo "Starting the flash process..."
echo "  "
cd /data/openpilot/selfdrive/car/modules/radarFlasher
PYTHONPATH=/data/openpilot 
./patch.py 
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " FLASH PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo " Please hit Reboot to return to OP"
  echo "An error occurred during flashing. Exiting..." >&2
  exit 1
fi
echo "================================="
echo " FLASH PROCESS COMPLETED"
echo "================================="
echo " Please hit Reboot to return to OP"
exit 0
