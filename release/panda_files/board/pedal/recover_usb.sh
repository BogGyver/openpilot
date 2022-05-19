#!/usr/bin/env sh
set -e

DFU_UTIL="dfu-util"
PYTHONPATH=../../.. python3 enter_usb_dfu_mode.py
$DFU_UTIL -d 0483:df11 -a 0 -s 0x08004000 -D ../obj/pedal.bin.signed
$DFU_UTIL -d 0483:df11 -a 0 -s 0x08000000:leave -D ../obj/bootstub.pedal.bin
