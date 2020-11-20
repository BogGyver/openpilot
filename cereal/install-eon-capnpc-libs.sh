#!/bin/sh
if [ -f /data/data/com.termux/files/usr/bin/capnpc-c ]; then
  exit 0
fi

if [ -n "`mount | grep '/system ' | grep 'ro'`" ]; then
    SYSTEM_WAS_RO="true"
    echo "Mounting read-write"
    mount -o rw,remount /system
fi
cd /
tar xvfz /data/openpilot/phonelibs/capnp-c/eon-capnpc.tgz

if [ -n "$SYSTEM_WAS_RO" ]; then
    echo "Re-mounting read-only"
    mount -o ro,remount /system
fi
