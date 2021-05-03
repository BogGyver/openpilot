#!/usr/bin/env sh
set -e

cd ..
GATEWAY=1 scons -u
cd gateway

../../tests/gateway/enter_canloader.py ../obj/panda.bin.signed
