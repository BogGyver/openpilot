#pragma once

#define DEGREES_TO_RADIANS 0.017453292519943295
#define RADIANS_TO_DEGREES (1.0 / DEGREES_TO_RADIANS)

#define MAX_ANGLE_OFFSET (10.0 * DEGREES_TO_RADIANS)
#define MAX_ANGLE_OFFSET_TH  (9.0 * DEGREES_TO_RADIANS)
#define MIN_STIFFNESS  0.5
#define MAX_STIFFNESS  2.0
#define MIN_SR  0.5
#define MAX_SR  2.0
#define MIN_SR_TH  0.55
#define MAX_SR_TH  1.9

#define FREQUENCY 100.0 //BB learner is called at 100 Hz
#define MAX_ACCEL 0.2 //BB maximum acceleration m/s^2 accepted before ignoring the calculations 

class ParamsLearner {
  double cF0, cR0;
  double aR, aF;
  double l, m;

  double min_sr, max_sr, min_sr_th, max_sr_th;
  double alpha1, alpha2, alpha3, alpha4;
  double cs_sr;

public:
  double ao;
  double slow_ao;
  double x, sR;
  double prev_u; //BB previous speed so we only learn when speed is constant between iterations

  ParamsLearner(cereal::CarParams::Reader car_params,
                double angle_offset,
                double stiffness_factor,
                double steer_ratio,
                double learning_rate);

  bool update(double psi, double u, double sa);
};
