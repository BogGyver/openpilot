#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>

#ifdef QCOM
#include <eigen3/Eigen/Dense>
#else
#include <Eigen/Dense>
#endif

#include "common/timing.h"
#include "driving.h"

#define MODEL_WIDTH 512
#define MODEL_HEIGHT 256
#define MODEL_NAME "driving_model_dlc"

#define LEAD_MDN_N 5 // probs for 5 groups
#define MDN_VALS 4 // output xyva for each lead group
#define SELECTION 3 //output 3 group (lead now, in 2s and 6s)
#define MDN_GROUP_SIZE 11
#define SPEED_BUCKETS 100
#define OUTPUT_SIZE ((MODEL_PATH_DISTANCE*2) + (2*(MODEL_PATH_DISTANCE*2 + 1)) + MDN_GROUP_SIZE*LEAD_MDN_N + SELECTION)
#ifdef TEMPORAL
  #define TEMPORAL_SIZE 512
#else
  #define TEMPORAL_SIZE 0
#endif

// #define DUMP_YUV

Eigen::Matrix<float, MODEL_PATH_DISTANCE, POLYFIT_DEGREE> vander;

void model_init(ModelState* s, cl_device_id device_id, cl_context context, int temporal) {
  model_input_init(&s->in, MODEL_WIDTH, MODEL_HEIGHT, device_id, context);
  const int output_size = OUTPUT_SIZE + TEMPORAL_SIZE;
  s->output = (float*)malloc(output_size * sizeof(float));
  memset(s->output, 0, output_size * sizeof(float));
  s->m = new DefaultRunModel("../../models/driving_model.dlc", s->output, output_size);
#ifdef TEMPORAL
  assert(temporal);
  s->m->addRecurrent(&s->output[OUTPUT_SIZE], TEMPORAL_SIZE);
#endif

  // Build Vandermonde matrix
  for(int i = 0; i < MODEL_PATH_DISTANCE; i++) {
    for(int j = 0; j < POLYFIT_DEGREE; j++) {
      vander(i, j) = pow(i, POLYFIT_DEGREE-j-1);
    }
  }
}

ModelData model_eval_frame(ModelState* s, cl_command_queue q,
                           cl_mem yuv_cl, int width, int height,
                           mat3 transform, void* sock) {
  struct {
    float *path;
    float *left_lane;
    float *right_lane;
    float *lead;
    float *speed;
  } net_outputs = {NULL};

  //for (int i = 0; i < OUTPUT_SIZE + TEMPORAL_SIZE; i++) { printf("%f ", s->output[i]); } printf("\n");

  float *net_input_buf = model_input_prepare(&s->in, q, yuv_cl, width, height, transform);

  #ifdef DUMP_YUV
    FILE *dump_yuv_file = fopen("/sdcard/dump.yuv", "wb");
    fwrite(net_input_buf, MODEL_HEIGHT*MODEL_WIDTH*3/2, sizeof(float), dump_yuv_file);
    fclose(dump_yuv_file);
    assert(1==2);
  #endif

  //printf("readinggggg \n");
  //FILE *f = fopen("goof_frame", "r");
  //fread(net_input_buf, sizeof(float), MODEL_HEIGHT*MODEL_WIDTH*3/2, f);
  //fclose(f);
  //sleep(1);
  //printf("%i \n",OUTPUT_SIZE);
  //printf("%i \n",MDN_GROUP_SIZE);
  s->m->execute(net_input_buf);

  // net outputs
  net_outputs.path = &s->output[0];
  net_outputs.left_lane = &s->output[MODEL_PATH_DISTANCE*2];
  net_outputs.right_lane = &s->output[MODEL_PATH_DISTANCE*2 + MODEL_PATH_DISTANCE*2 + 1];
  net_outputs.lead = &s->output[MODEL_PATH_DISTANCE*2 + (MODEL_PATH_DISTANCE*2 + 1)*2];
  //net_outputs.speed = &s->output[OUTPUT_SIZE - SPEED_BUCKETS];

  ModelData model = {0};

  for (int i=0; i<MODEL_PATH_DISTANCE; i++) {
    model.path.points[i] = net_outputs.path[i];
    model.left_lane.points[i] = net_outputs.left_lane[i] + 1.8;
    model.right_lane.points[i] = net_outputs.right_lane[i] - 1.8;
    model.path.stds[i] = softplus(net_outputs.path[MODEL_PATH_DISTANCE + i]);
    model.left_lane.stds[i] = softplus(net_outputs.left_lane[MODEL_PATH_DISTANCE + i]);
    model.right_lane.stds[i] = softplus(net_outputs.right_lane[MODEL_PATH_DISTANCE + i]);
  }

  model.path.std = softplus(net_outputs.path[MODEL_PATH_DISTANCE + MODEL_PATH_DISTANCE/4]);
  model.left_lane.std = softplus(net_outputs.left_lane[MODEL_PATH_DISTANCE + MODEL_PATH_DISTANCE/4]);
  model.right_lane.std = softplus(net_outputs.right_lane[MODEL_PATH_DISTANCE + MODEL_PATH_DISTANCE/4]);

  model.path.prob = 1.;
  model.left_lane.prob = sigmoid(net_outputs.left_lane[MODEL_PATH_DISTANCE*2]);
  model.right_lane.prob = sigmoid(net_outputs.right_lane[MODEL_PATH_DISTANCE*2]);

  poly_fit(model.path.points, model.path.stds, model.path.poly);
  poly_fit(model.left_lane.points, model.left_lane.stds, model.left_lane.poly);
  poly_fit(model.right_lane.points, model.right_lane.stds, model.right_lane.poly);

  const double max_dist = 140.0;
  const double max_rel_vel = 10.0;
  // Every output distribution from the MDN includes the probabilties
  // of it representing a current lead car, a lead car in 2s
  // or a lead car in 4s

  // Find the distribution that corresponds to the current lead
  int mdn_max_idx = 0;
  for (int i=1; i<LEAD_MDN_N; i++) {
    if (net_outputs.lead[i*MDN_GROUP_SIZE + 8] > net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + 8]) {
      mdn_max_idx = i;
    }
  }
  model.lead.prob = sigmoid(net_outputs.lead[LEAD_MDN_N*MDN_GROUP_SIZE]);
  model.lead.dist = net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE] * max_dist;
  model.lead.std = softplus(net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + MDN_VALS]) * max_dist;
  model.lead.rel_y = net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + 1];
  model.lead.rel_y_std = softplus(net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + MDN_VALS + 1]);
  model.lead.rel_v = net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + 2] * max_rel_vel;
  model.lead.rel_v_std = softplus(net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + MDN_VALS + 2]) * max_rel_vel;
  model.lead.rel_a = net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + 3];
  model.lead.rel_a_std = softplus(net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + MDN_VALS + 3]);

  // Find the distribution that corresponds to the lead in 2s
  mdn_max_idx = 0;
  for (int i=1; i<LEAD_MDN_N; i++) {
    if (net_outputs.lead[i*MDN_GROUP_SIZE + 9] > net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + 9]) {
      mdn_max_idx = i;
    }
  }
  model.lead_future.prob = sigmoid(net_outputs.lead[LEAD_MDN_N*MDN_GROUP_SIZE + 1]);
  model.lead_future.dist = net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE] * max_dist;
  model.lead_future.std = softplus(net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + MDN_VALS]) * max_dist;
  model.lead_future.rel_y = net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + 1];
  model.lead_future.rel_y_std = softplus(net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + MDN_VALS + 1]);
  model.lead_future.rel_v = net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + 2] * max_rel_vel;
  model.lead_future.rel_v_std = softplus(net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + MDN_VALS + 2]) * max_rel_vel;
  model.lead_future.rel_a = net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + 3];
  model.lead_future.rel_a_std = softplus(net_outputs.lead[mdn_max_idx*MDN_GROUP_SIZE + MDN_VALS + 3]);


  // get speed percentiles numbers represent 5th, 15th, ... 95th percentile
  for (int i=0; i < SPEED_PERCENTILES; i++) {
    model.speed[i] = ((float) SPEED_BUCKETS)/2.0;
  }
  //float sum = 0;
  //for (int idx = 0; idx < SPEED_BUCKETS; idx++) {
  //  sum += net_outputs.speed[idx];
  //  int idx_percentile = (sum + .05) * SPEED_PERCENTILES;
  //  if (idx_percentile < SPEED_PERCENTILES ){
  //    model.speed[idx_percentile] = ((float)idx)/2.0;
  //  }
  //}
  // make sure no percentiles are skipped
  //for (int i=SPEED_PERCENTILES-1; i > 0; i--){
  //  if (model.speed[i-1] > model.speed[i]){
  //    model.speed[i-1] = model.speed[i];
  //  }
  //}
  return model;
}

void model_free(ModelState* s) {
  free(s->output);
  model_input_free(&s->in);
  delete s->m;
}

void poly_fit(float *in_pts, float *in_stds, float *out) {
  // References to inputs
  Eigen::Map<Eigen::Matrix<float, MODEL_PATH_DISTANCE, 1> > pts(in_pts, MODEL_PATH_DISTANCE);
  Eigen::Map<Eigen::Matrix<float, MODEL_PATH_DISTANCE, 1> > std(in_stds, MODEL_PATH_DISTANCE);
  Eigen::Map<Eigen::Matrix<float, POLYFIT_DEGREE, 1> > p(out, POLYFIT_DEGREE);

  // Build Least Squares equations
  Eigen::Matrix<float, MODEL_PATH_DISTANCE, POLYFIT_DEGREE> lhs = vander.array().colwise() / std.array();
  Eigen::Matrix<float, MODEL_PATH_DISTANCE, 1> rhs = pts.array() / std.array();

  // Improve numerical stability
  Eigen::Matrix<float, POLYFIT_DEGREE, 1> scale = 1. / (lhs.array()*lhs.array()).sqrt().colwise().sum();
  lhs = lhs * scale.asDiagonal();

  // Solve inplace
  Eigen::ColPivHouseholderQR<Eigen::Ref<Eigen::MatrixXf> > qr(lhs);
  p = qr.solve(rhs);

  // Apply scale to output
  p = p.transpose() * scale.asDiagonal();
}
