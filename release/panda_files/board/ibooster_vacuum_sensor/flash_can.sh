#!/usr/bin/env sh
set -e

cd ..
IVS=1 scons -u
cd ibooster_vacuum_sensor

./enter_canloader.py ../obj/ivs.bin.signed
