#!/usr/bin/env python


from collections import namedtuple
import json
import os
import sys
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit
from pathlib import Path
import tensorflow as tf
import tensorrt as trt

HostDeviceMemory = namedtuple('HostDeviceMemory', 'host_memory device_memory')

def allocate_buffers(cls, engine):
  inputs = []
  outputs = []
  bindings = []
  stream = cuda.Stream()
  for binding in engine:
    size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
    dtype = trt.nptype(engine.get_binding_dtype(binding))
    # Allocate host and device buffers
    host_memory = cuda.pagelocked_empty(size, dtype)
    device_memory = cuda.mem_alloc(host_memory.nbytes)
    bindings.append(int(device_memory))
    if engine.binding_is_input(binding):
        inputs.append(HostDeviceMemory(host_memory, device_memory))
    else:
        outputs.append(HostDeviceMemory(host_memory, device_memory))

  return inputs, outputs, bindings, stream

def infer(cls, context, bindings, inputs, outputs, stream, batch_size=1):
  # Transfer input data to the GPU.
  [cuda.memcpy_htod_async(inp.device_memory, inp.host_memory, stream) for inp in inputs]
  # Run inference.
  context.execute_async(batch_size=batch_size, bindings=bindings, stream_handle=stream.handle)
  # Transfer predictions back from the GPU.
  [cuda.memcpy_dtoh_async(out.host_memory, out.device_memory, stream) for out in outputs]
  # Synchronize the stream
  stream.synchronize()
  # Return only the host outputs.
  return [out.host_memory for out in outputs]



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

def run_loop(tf_sess,input_tensor_name,output_tensor_name):
  input_tensor = tf_sess.graph.get_tensor_by_name(input_tensor_name)
  output_tensor = tf_sess.graph.get_tensor_by_name(output_tensor_name)
  isize = input_tensor.size
  osize = output_tensor.size

if __name__ == "__main__":
  print(tf.__version__, file=sys.stderr)
  model_file = Path(sys.argv[1])
  uff_model = Path('%s/%s/%s.trt' % (model_file.parent.as_posix(),'trt',model_file.stem))
  metadata_path = Path('%s/%s/%s.metadata' % (model_file.parent.as_posix(), 'trt', model_file.stem))
  with open(metadata_path.as_posix(), 'r') as metadata, trt.Builder() as builder, builder.create_network() as network, trt.UffParser() as parser:
     metadata = json.loads(metadata.read())
     # Configure inputs and outputs
     print('Configuring I/O')
     input_names = metadata['input_names']
     output_names = metadata['output_names']
     for name in input_names:
         parser.register_input(name, (self.cfg.TARGET_D, self.cfg.TARGET_H, self.cfg.TARGET_W))

     for name in output_names:
         parser.register_output(name)
     # Parse network
     print('Parsing TensorRT Network')
     parser.parse(uff_model.as_posix(), network)
     print('Building CUDA Engine')
     engine = builder.build_cuda_engine(network)
     # Allocate buffers
     print('Allocating Buffers')
     inputs, outputs, bindings, stream = allocate_buffers(engine)
     print('Ready')

  isize = inputs.size
  osize = output.size

  print("Ready to run keras model %d -> %d " % (isize,osize), file=sys.stderr)
  while 1:
    # check parent process, if ppid is 1, then modeld is no longer running and the runner should exit.
    if os.getppid() == 1:
      print("exiting due to Parent PID", file=sys.stderr)  
      break
    idata = read(isize).reshape((1, isize))
    with engine.create_execution_context() as context:
      ret = infer(context=context, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)
    write(ret)
        
