#!/usr/bin/env sh
DFU_UTIL="dfu-util"

echo "================================="
echo "RECOVER PROCESS STARTED"
echo "================================="
echo "  "
echo "Starting the recover process..."
echo "  "
if [ -f "/data/openpilot/prebuilt" ]; then
  echo "Prebuilt code... just flashing..."
else
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
fi
PYTHONPATH=.. python3 -c "from python import Panda; Panda().reset(enter_bootstub=True); Panda().reset(enter_bootloader=True)" || true
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " RECOVER PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo "  "
  exit 1
fi
sleep 1
$DFU_UTIL -d 0483:df11 -a 0 -s 0x08004000 -D obj/panda.bin.signed
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " RECOVER PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo "  "
  exit 1
fi
$DFU_UTIL -d 0483:df11 -a 0 -s 0x08000000:leave -D obj/bootstub.panda.bin
ret=$?
if [ $ret -ne 0 ]; then
  echo "================================="
  echo " RECOVER PROCESS FAILED"
  echo "================================="
  echo " Please check logs above for errors"
  echo "  "
  exit 1
fi
echo "================================="
echo " RECOVER PROCESS COMPLETED"
echo "================================="
echo " Please hit Reboot to return to OP"
exit 0
