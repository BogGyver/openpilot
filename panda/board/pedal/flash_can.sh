#!/usr/bin/env sh
set -e

cd ..
PEDAL=1 scons -u -j$(nproc)
cd pedal

./enter_canloader.py ../obj/pedal.bin.signed
