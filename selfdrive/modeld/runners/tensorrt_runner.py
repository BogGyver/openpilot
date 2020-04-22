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
import argparse
from onnx import ModelProto

HostDeviceMemory = namedtuple('HostDeviceMemory', 'host_memory device_memory')

def allocate_buffers(engine, batch_size, data_type, insize,outsize):

   """
   This is the function to allocate buffers for input and output in the device
   Args:
      engine : The path to the TensorRT engine. 
      batch_size : The batch size for execution time.
      data_type: The type of the data for input and output, for example trt.float32. 
   
   Output:
      h_input_1: Input in the host.
      d_input_1: Input in the device. 
      h_output_1: Output in the host. 
      d_output_1: Output in the device. 
      stream: CUDA stream.

   """

   # Determine dimensions and create page-locked memory buffers (which won't be swapped to disk) to hold host inputs/outputs.
   h_input_1 = cuda.pagelocked_empty(batch_size * insize, dtype=trt.float32)
   h_output = cuda.pagelocked_empty(batch_size * outsize, dtype=trt.float32) #trt.nptype(data_type))
   # Allocate device memory for inputs and outputs.
   d_input_1 = cuda.mem_alloc(h_input_1.nbytes)

   d_output = cuda.mem_alloc(h_output.nbytes)
   # Create a stream in which to copy inputs/outputs and run inference.
   stream = cuda.Stream()
   return h_input_1, d_input_1, h_output, d_output, stream 


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

def load_data_to_buffer(idata, pagelocked_buffer):
   preprocessed = np.asarray(idata).ravel()
   np.copyto(pagelocked_buffer, preprocessed)

if __name__ == "__main__":
  print(tf.__version__, file=sys.stderr)
  model_file = Path(sys.argv[1])
  onnx_model = Path('%s/%s.onnx' % (model_file.parent.as_posix(),model_file.stem))
  print(" ONNX  [%s] " % (onnx_model),file=sys.stderr)
  logger = trt.Logger(trt.Logger.WARNING)
  batch_size = 1 
   
  #model = ModelProto()
  #with open(onnx_model, "rb") as f:
  #  model.ParseFromString(f.read())
  
  d0 = 394250 #model.graph.input[0].type.tensor_type.shape.dim[1].dim_value
  d1 = 1 #model.graph.input[0].type.tensor_type.shape.dim[2].dim_value
  d2 = 1 # model.graph.input[0].type.tensor_type.shape.dim[3].dim_value
  o0 = 2895 #model.graph.output[0].type.tensor_type.shape.dim[1].dim_value
  o1 = 1 #model.graph.output[0].type.tensor_type.shape.dim[2].dim_value
  o2 = 1 #model.graph.output[0].type.tensor_type.shape.dim[3].dim_value
  out_size = o0 * o1 * o2
  shape = [batch_size , d0, d1 ,d2]
  print(" Input shape ",shape,' Output size ',out_size,file=sys.stderr)
  trt.init_libnvinfer_plugins(logger, '')
  builder = trt.Builder(logger)
  network = builder.create_network()
  print('Parsing TensorRT Network',file=sys.stderr)
  parser = trt.OnnxParser(network, logger)
  builder.max_workspace_size = (256 << 20)
  with open(onnx_model, 'rb') as model:
    parser.parse(model.read())
  network.get_input(0).shape = shape
  print('Building CUDA Engine',file=sys.stderr)
  engine = builder.build_cuda_engine(network)
  print('Allocating buffers ',file=sys.stderr)
  h_input, d_input, h_output, d_output, stream = allocate_buffers(engine, 1, trt.float32,d0,o0)
  print("Ready to run TensorRT CUDA model",  file=sys.stderr)
  context = engine.create_execution_context()
  while 1:
    # check parent process, if ppid is 1, then modeld is no longer running and the runner should exit.
    if os.getppid() == 1:
      print("exiting due to Parent PID", file=sys.stderr)  
      break
    idata = read(isize).reshape((1, isize))
    load_data_to_buffer(idata, h_input_1)
    cuda.memcpy_htod_async(d_input_1, h_input_1, stream)
    context.execute(batch_size=1, bindings=[int(d_input_1), int(d_output)])
    cuda.memcpy_dtoh_async(h_output, d_output, stream)
    stream.synchronize()
    ret = h_output.reshape((batch_size,-1, 1, out_size))
    write(ret)
  #TODO: clean memory and free resources
        
