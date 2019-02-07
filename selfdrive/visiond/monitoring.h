#ifndef MONITORING_H
#define MONITORING_H

#include "commonmodel.h"
#include "snpemodel.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OUTPUT_SIZE 7

typedef struct MonitoringResult {
  float vs[6];
  float std;
} MonitoringResult;

typedef struct MonitoringState {
  ModelInput in;
  SNPEModel *m;
  float output[OUTPUT_SIZE];
} MonitoringState;

void monitoring_init(MonitoringState* s, cl_device_id device_id, cl_context context);
MonitoringResult monitoring_eval_frame(MonitoringState* s, cl_command_queue q, cl_mem yuv_cl, int width, int height);
void monitoring_free(MonitoringState* s);

#ifdef __cplusplus
}
#endif

#endif
