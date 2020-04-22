#!/usr/bin/env python

import os
import sys
import numpy as np
import onnx
import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as cuda
from pathlib import Path
from onnx import ModelProto

TRT_LOGGER = trt.Logger(trt.Logger.VERBOSE)
trt_runtime = trt.Runtime(TRT_LOGGER)
trt.init_libnvinfer_plugins(TRT_LOGGER, '')

def build_engine(onnx_path):
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network() 
    parser  = trt.OnnxParser(network, TRT_LOGGER)
    builder.max_workspace_size = 1 << 20  # 1024MB
    builder.max_batch_size = 1
#    builder.fp16_mode = True
    print(onnx_path, file=sys.stderr)
    with open(onnx_path, 'rb') as model:
      parser.parse(model.read())
    out_size = 2895
    isize = network.get_input(0).shape
    last_layer = network.get_layer(network.num_layers - 1)
    network.mark_output(last_layer.get_output(0))
    print(network, file=sys.stderr)
    engine = builder.build_cuda_engine(network)
    return engine

def save_engine(engine, file_name):
   buf = engine.serialize()
   with open(file_name, 'wb') as f:
       f.write(buf)

def load_engine(trt_runtime, engine_path):
   with open(engine_path, 'rb') as f:
       engine_data = f.read()
   engine = trt_runtime.deserialize_cuda_engine(engine_data)
   return engine

def alloc_buf(engine, isize, osize):
    # host cpu mem
    in_cpu = cuda.pagelocked_empty(isize, dtype=np.float32)
    out_cpu = cuda.pagelocked_empty(osize*10000, dtype=np.float32)
    # allocate gpu mem
    in_gpu = cuda.mem_alloc(in_cpu.nbytes)
    out_gpu = cuda.mem_alloc(out_cpu.nbytes)
    return in_cpu, out_cpu, in_gpu, out_gpu

def predict(context, in_cpu, out_cpu, in_gpu, out_gpu):
    cuda.memcpy_htod(in_gpu, in_cpu)
    context.execute(1, bindings=[int(in_gpu), int(out_gpu)])
    cuda.memcpy_dtoh(out_cpu, out_gpu)

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
  print(d, file=sys.stderr)
  os.write(1, d.tobytes())

def run_loop(engine, context, in_cpu, out_cpu, in_gpu, out_gpu, isize, osize):
  print("ready to run keras model %d -> %d" % (isize, osize), file=sys.stderr)
  while 1:
    # check parent process, if ppid is 1, then modeld is no longer running and the runner should exit.
    if os.getppid() == 1:
      print("exiting due to Parent PID", file=sys.stderr)  
      break
    idata = read(isize).reshape((1, isize))
    predict(context, idata, out_cpu, in_gpu, out_gpu)
    write(out_cpu)

if __name__ == "__main__":
  model_path = Path(sys.argv[1])
  onnx_model = Path(f"{model_path.parent.as_posix()}/{model_path.stem}.onnx")
  print(onnx_model, file=sys.stderr)
  isize = 394250
  osize = 2895
  #if os.path.isfile(f"{onnx_model}.engine"):
  #  load_engine(trt_runtime, f"{onnx_model}.engine")
  #else:
  engine = build_engine(onnx_model)
  #save_engine(engine, f"{model_path}.engine")
  context = engine.create_execution_context()
  in_cpu, out_cpu, in_gpu, out_gpu = alloc_buf(engine, isize, osize)
  run_loop(engine, context, in_cpu, out_cpu, in_gpu, out_gpu, isize, osize)

