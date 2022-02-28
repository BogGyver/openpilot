#! /bin/sh
echo "Starting the flash process..."
cd /data/openpilot/selfdrive/car/modules/teslaEpasFlasher
PYTHONPATH=/data/openpilot 
./patch.py 
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " FLASH PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo " Please hit Reboot to return to OP"
  exit(1)
fi
echo "================================="
echo " FLASH PROCESS COMPLETED"
echo "================================="
echo " Please hit Reboot to return to OP"