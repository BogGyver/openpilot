#!/bin/sh

gcc -I ../../include \
  -I ~/one/phonelibs/android_system_core/include -I ~/one/phonelibs/opencl/include \
  -I ~/one/selfdrive/visiond/cameras \
  -I /usr/local/lib -I /usr/lib/aarch64-linux-gnu -I /usr/lib -I /data/op_rk3399_setup/external/snpe/lib/lib -I /data/data/com.termux/files/usr/lib -I /data/op_rk3399_setup/support_files/lib

  test.c ../../cameras/camera_qcom.c \
  -l:libczmq.a -l:libzmq.a -lgnustl_shared -lm -llog -lcutils \
  -l:libcapn.a -l:libcapnp.a -l:libkj.a \
  ~/one/cereal/gen/c/log.capnp.o

