#!/usr/bin/env python3

import os

import json
from pathlib import Path
import tensorflow as tf
import tensorflow.keras as keras
import numpy as np
from tensorflow.keras.models import Model
from tensorflow.keras.models import model_from_json, load_model
import sys
import numpy as np
in_model = os.path.expanduser(sys.argv[1])
output = os.path.expanduser(sys.argv[2])
input_path = Path(in_model)
output_p = '%s/%s' % (output, input_path.stem)
output_path = Path(output_p)
output_meta = Path('%s/%s.metadata' % (output_path.parent.as_posix(), output_path.stem))


# Reset session
tf.keras.backend.clear_session()
tf.keras.backend.set_learning_phase(0)

model = tf.keras.models.load_model(in_model, compile=False)
session = tf.keras.backend.get_session()

bs = [int(np.product(ii.shape[1:])) for ii in model.inputs]
ri = keras.layers.Input((sum(bs),))
tii = []
acc = 0
for i, ii in enumerate(model.inputs):
  ti = keras.layers.Lambda(lambda x: x[:,acc:acc+bs[i]], output_shape=(1, bs[i]))(ri)
  acc += bs[i]
  tr = keras.layers.Reshape(ii.shape[1:])(ti)
  tii.append(tr)
no = keras.layers.Concatenate()(model(tii))
model = Model(inputs=ri, outputs=[no])

input_names = sorted([layer.op.name for layer in model.inputs])
output_names = sorted([layer.op.name for layer in model.outputs])

# Store additional information in metadata, useful when creating a TensorRT network
meta = {'input_names': input_names, 'output_names': output_names}

model.summary()
tf.saved_model.save(model,output_p)

