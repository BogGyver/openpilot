#!/usr/bin/env sh
echo "================================="
echo "FLASH PROCESS STARTED"
echo "================================="
echo "  "
echo "Starting the flash process..."
echo "  "

scons -u
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " COMPILE PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo "  "
  echo "An error occurred during flashing. Exiting..." >&2
  exit 1
fi
PYTHONPATH=.. python3 -c "from python import Panda; Panda().flash('obj/panda.bin.signed')"
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " FLASH PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo "  "
  echo "An error occurred during flashing. Exiting..." >&2
  exit 1
fi
echo "================================="
echo " FLASH PROCESS COMPLETED"
echo "================================="
echo " Please hit Reboot to return to OP"
exit 0
