#!/usr/bin/env python
# TODO: why are the keras models saved with python 2?

import tensorflow as tf
import os
import sys
import tensorflow.keras as keras
import numpy as np
from tensorflow.keras.models import Model
from tensorflow.keras.models import model_from_json, load_model

def read(sz):
  dd = []
  gt = 0
  while gt < sz*4:
    st = os.read(0, sz*4 - gt)
    assert(len(st) > 0)
    dd.append(st)
    gt += len(st)
  return np.fromstring(b''.join(dd), dtype=np.float32)

def write(d):
  os.write(1, d.tobytes())

def run_loop(m):
  isize = m.inputs[0].shape[1]
  osize = m.outputs[0].shape[1]
  print("ready to run keras model %d -> %d" % (isize, osize), file=sys.stderr)
  while 1:
    # check parent process, if ppid is 1, then modeld is no longer running and the runner should exit.
    if os.getppid() == 1:
      print("exiting due to Parent PID", file=sys.stderr)  
      break
    idata = read(isize).reshape((1, isize))
    ret = m.predict_on_batch(idata)
    write(ret)

if __name__ == "__main__":
  print(tf.__version__, file=sys.stderr)
  # limit gram alloc
  gpus = tf.config.experimental.list_physical_devices('GPU')
  if len(gpus) > 0:
    if os.path.splitext(os.path.basename(sys.argv[1]))[0]== "supercombo":
      tf.config.experimental.set_virtual_device_configuration(gpus[0], [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=1772)])
    else:
      tf.config.experimental.set_virtual_device_configuration(gpus[0], [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=306)])
  with open(f"{os.path.splitext(sys.argv[1])[0]}.model.keras", "r") as json_file:
    m = model_from_json(json_file.read())
    m.load_weights(f"{os.path.splitext(sys.argv[1])[0]}.weights.keras")

  bs = [int(np.product(ii.shape[1:])) for ii in m.inputs]
  ri = keras.layers.Input((sum(bs),))
  tii = []
  acc = 0
  for i, ii in enumerate(m.inputs):
    ti = keras.layers.Lambda(lambda x: x[:,acc:acc+bs[i]], output_shape=(1, bs[i]))(ri)
    acc += bs[i]
    tr = keras.layers.Reshape(ii.shape[1:])(ti)
    tii.append(tr)
  no = keras.layers.Concatenate()(m(tii))
  m = Model(inputs=ri, outputs=[no])
  run_loop(m)

