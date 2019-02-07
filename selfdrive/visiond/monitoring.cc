#include "monitoring.h"
#include "common/mat.h"

#define MODEL_WIDTH 320
#define MODEL_HEIGHT 160
extern const uint8_t monitoring_model_data[] asm("_binary_monitoring_model_dlc_start");
extern const uint8_t monitoring_model_end[] asm("_binary_monitoring_model_dlc_end");
const size_t monitoring_model_size = monitoring_model_end - monitoring_model_data;


void monitoring_init(MonitoringState* s, cl_device_id device_id, cl_context context) {
  model_input_init(&s->in, MODEL_WIDTH, MODEL_HEIGHT, device_id, context);
  s->m = new SNPEModel(monitoring_model_data, monitoring_model_size, (float*)&s->output, OUTPUT_SIZE);
}

MonitoringResult monitoring_eval_frame(MonitoringState* s, cl_command_queue q,
                           cl_mem yuv_cl, int width, int height) {
  const mat3 front_frame_from_scaled_frame = (mat3){{
    width/426.0f,          0.0, 0.0,
             0.0,height/320.0f, 0.0,
             0.0,          0.0, 1.0,
  }};

  const mat3 scaled_frame_from_cropped_frame = (mat3){{
      1.0, 0.0, 426.0-160.0,
      0.0, 1.0,         0.0,
      0.0, 0.0,         1.0,
  }};

  const mat3 transpose = (mat3){{
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, 1.0,
  }};

  const mat3 front_frame_from_cropped_frame = matmul3(front_frame_from_scaled_frame, scaled_frame_from_cropped_frame);
  const mat3 front_frame_from_monitoring_frame = matmul3(front_frame_from_cropped_frame, transpose);

  float *net_input_buf = model_input_prepare(&s->in, q, yuv_cl, width, height, front_frame_from_monitoring_frame);
  s->m->execute(net_input_buf);

  MonitoringResult ret = {0};
  memcpy(ret.vs, s->output, sizeof(ret.vs));
  ret.std = sqrtf(2.f) / s->output[6];

  return ret;
}


void monitoring_free(MonitoringState* s) {
  model_input_free(&s->in);
  delete s->m;
}

