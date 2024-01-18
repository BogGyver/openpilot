#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_198445823836928788) {
   out_198445823836928788[0] = delta_x[0] + nom_x[0];
   out_198445823836928788[1] = delta_x[1] + nom_x[1];
   out_198445823836928788[2] = delta_x[2] + nom_x[2];
   out_198445823836928788[3] = delta_x[3] + nom_x[3];
   out_198445823836928788[4] = delta_x[4] + nom_x[4];
   out_198445823836928788[5] = delta_x[5] + nom_x[5];
   out_198445823836928788[6] = delta_x[6] + nom_x[6];
   out_198445823836928788[7] = delta_x[7] + nom_x[7];
   out_198445823836928788[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7598262341626218705) {
   out_7598262341626218705[0] = -nom_x[0] + true_x[0];
   out_7598262341626218705[1] = -nom_x[1] + true_x[1];
   out_7598262341626218705[2] = -nom_x[2] + true_x[2];
   out_7598262341626218705[3] = -nom_x[3] + true_x[3];
   out_7598262341626218705[4] = -nom_x[4] + true_x[4];
   out_7598262341626218705[5] = -nom_x[5] + true_x[5];
   out_7598262341626218705[6] = -nom_x[6] + true_x[6];
   out_7598262341626218705[7] = -nom_x[7] + true_x[7];
   out_7598262341626218705[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2149329292292161718) {
   out_2149329292292161718[0] = 1.0;
   out_2149329292292161718[1] = 0;
   out_2149329292292161718[2] = 0;
   out_2149329292292161718[3] = 0;
   out_2149329292292161718[4] = 0;
   out_2149329292292161718[5] = 0;
   out_2149329292292161718[6] = 0;
   out_2149329292292161718[7] = 0;
   out_2149329292292161718[8] = 0;
   out_2149329292292161718[9] = 0;
   out_2149329292292161718[10] = 1.0;
   out_2149329292292161718[11] = 0;
   out_2149329292292161718[12] = 0;
   out_2149329292292161718[13] = 0;
   out_2149329292292161718[14] = 0;
   out_2149329292292161718[15] = 0;
   out_2149329292292161718[16] = 0;
   out_2149329292292161718[17] = 0;
   out_2149329292292161718[18] = 0;
   out_2149329292292161718[19] = 0;
   out_2149329292292161718[20] = 1.0;
   out_2149329292292161718[21] = 0;
   out_2149329292292161718[22] = 0;
   out_2149329292292161718[23] = 0;
   out_2149329292292161718[24] = 0;
   out_2149329292292161718[25] = 0;
   out_2149329292292161718[26] = 0;
   out_2149329292292161718[27] = 0;
   out_2149329292292161718[28] = 0;
   out_2149329292292161718[29] = 0;
   out_2149329292292161718[30] = 1.0;
   out_2149329292292161718[31] = 0;
   out_2149329292292161718[32] = 0;
   out_2149329292292161718[33] = 0;
   out_2149329292292161718[34] = 0;
   out_2149329292292161718[35] = 0;
   out_2149329292292161718[36] = 0;
   out_2149329292292161718[37] = 0;
   out_2149329292292161718[38] = 0;
   out_2149329292292161718[39] = 0;
   out_2149329292292161718[40] = 1.0;
   out_2149329292292161718[41] = 0;
   out_2149329292292161718[42] = 0;
   out_2149329292292161718[43] = 0;
   out_2149329292292161718[44] = 0;
   out_2149329292292161718[45] = 0;
   out_2149329292292161718[46] = 0;
   out_2149329292292161718[47] = 0;
   out_2149329292292161718[48] = 0;
   out_2149329292292161718[49] = 0;
   out_2149329292292161718[50] = 1.0;
   out_2149329292292161718[51] = 0;
   out_2149329292292161718[52] = 0;
   out_2149329292292161718[53] = 0;
   out_2149329292292161718[54] = 0;
   out_2149329292292161718[55] = 0;
   out_2149329292292161718[56] = 0;
   out_2149329292292161718[57] = 0;
   out_2149329292292161718[58] = 0;
   out_2149329292292161718[59] = 0;
   out_2149329292292161718[60] = 1.0;
   out_2149329292292161718[61] = 0;
   out_2149329292292161718[62] = 0;
   out_2149329292292161718[63] = 0;
   out_2149329292292161718[64] = 0;
   out_2149329292292161718[65] = 0;
   out_2149329292292161718[66] = 0;
   out_2149329292292161718[67] = 0;
   out_2149329292292161718[68] = 0;
   out_2149329292292161718[69] = 0;
   out_2149329292292161718[70] = 1.0;
   out_2149329292292161718[71] = 0;
   out_2149329292292161718[72] = 0;
   out_2149329292292161718[73] = 0;
   out_2149329292292161718[74] = 0;
   out_2149329292292161718[75] = 0;
   out_2149329292292161718[76] = 0;
   out_2149329292292161718[77] = 0;
   out_2149329292292161718[78] = 0;
   out_2149329292292161718[79] = 0;
   out_2149329292292161718[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7137556374309530724) {
   out_7137556374309530724[0] = state[0];
   out_7137556374309530724[1] = state[1];
   out_7137556374309530724[2] = state[2];
   out_7137556374309530724[3] = state[3];
   out_7137556374309530724[4] = state[4];
   out_7137556374309530724[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7137556374309530724[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7137556374309530724[7] = state[7];
   out_7137556374309530724[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8393538857825963338) {
   out_8393538857825963338[0] = 1;
   out_8393538857825963338[1] = 0;
   out_8393538857825963338[2] = 0;
   out_8393538857825963338[3] = 0;
   out_8393538857825963338[4] = 0;
   out_8393538857825963338[5] = 0;
   out_8393538857825963338[6] = 0;
   out_8393538857825963338[7] = 0;
   out_8393538857825963338[8] = 0;
   out_8393538857825963338[9] = 0;
   out_8393538857825963338[10] = 1;
   out_8393538857825963338[11] = 0;
   out_8393538857825963338[12] = 0;
   out_8393538857825963338[13] = 0;
   out_8393538857825963338[14] = 0;
   out_8393538857825963338[15] = 0;
   out_8393538857825963338[16] = 0;
   out_8393538857825963338[17] = 0;
   out_8393538857825963338[18] = 0;
   out_8393538857825963338[19] = 0;
   out_8393538857825963338[20] = 1;
   out_8393538857825963338[21] = 0;
   out_8393538857825963338[22] = 0;
   out_8393538857825963338[23] = 0;
   out_8393538857825963338[24] = 0;
   out_8393538857825963338[25] = 0;
   out_8393538857825963338[26] = 0;
   out_8393538857825963338[27] = 0;
   out_8393538857825963338[28] = 0;
   out_8393538857825963338[29] = 0;
   out_8393538857825963338[30] = 1;
   out_8393538857825963338[31] = 0;
   out_8393538857825963338[32] = 0;
   out_8393538857825963338[33] = 0;
   out_8393538857825963338[34] = 0;
   out_8393538857825963338[35] = 0;
   out_8393538857825963338[36] = 0;
   out_8393538857825963338[37] = 0;
   out_8393538857825963338[38] = 0;
   out_8393538857825963338[39] = 0;
   out_8393538857825963338[40] = 1;
   out_8393538857825963338[41] = 0;
   out_8393538857825963338[42] = 0;
   out_8393538857825963338[43] = 0;
   out_8393538857825963338[44] = 0;
   out_8393538857825963338[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8393538857825963338[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8393538857825963338[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8393538857825963338[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8393538857825963338[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8393538857825963338[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8393538857825963338[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8393538857825963338[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8393538857825963338[53] = -9.8000000000000007*dt;
   out_8393538857825963338[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8393538857825963338[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8393538857825963338[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8393538857825963338[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8393538857825963338[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8393538857825963338[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8393538857825963338[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8393538857825963338[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8393538857825963338[62] = 0;
   out_8393538857825963338[63] = 0;
   out_8393538857825963338[64] = 0;
   out_8393538857825963338[65] = 0;
   out_8393538857825963338[66] = 0;
   out_8393538857825963338[67] = 0;
   out_8393538857825963338[68] = 0;
   out_8393538857825963338[69] = 0;
   out_8393538857825963338[70] = 1;
   out_8393538857825963338[71] = 0;
   out_8393538857825963338[72] = 0;
   out_8393538857825963338[73] = 0;
   out_8393538857825963338[74] = 0;
   out_8393538857825963338[75] = 0;
   out_8393538857825963338[76] = 0;
   out_8393538857825963338[77] = 0;
   out_8393538857825963338[78] = 0;
   out_8393538857825963338[79] = 0;
   out_8393538857825963338[80] = 1;
}
void h_25(double *state, double *unused, double *out_1913767349031954091) {
   out_1913767349031954091[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3307554411264577435) {
   out_3307554411264577435[0] = 0;
   out_3307554411264577435[1] = 0;
   out_3307554411264577435[2] = 0;
   out_3307554411264577435[3] = 0;
   out_3307554411264577435[4] = 0;
   out_3307554411264577435[5] = 0;
   out_3307554411264577435[6] = 1;
   out_3307554411264577435[7] = 0;
   out_3307554411264577435[8] = 0;
}
void h_24(double *state, double *unused, double *out_1050780426769953054) {
   out_1050780426769953054[0] = state[4];
   out_1050780426769953054[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8054063738231086469) {
   out_8054063738231086469[0] = 0;
   out_8054063738231086469[1] = 0;
   out_8054063738231086469[2] = 0;
   out_8054063738231086469[3] = 0;
   out_8054063738231086469[4] = 1;
   out_8054063738231086469[5] = 0;
   out_8054063738231086469[6] = 0;
   out_8054063738231086469[7] = 0;
   out_8054063738231086469[8] = 0;
   out_8054063738231086469[9] = 0;
   out_8054063738231086469[10] = 0;
   out_8054063738231086469[11] = 0;
   out_8054063738231086469[12] = 0;
   out_8054063738231086469[13] = 0;
   out_8054063738231086469[14] = 1;
   out_8054063738231086469[15] = 0;
   out_8054063738231086469[16] = 0;
   out_8054063738231086469[17] = 0;
}
void h_30(double *state, double *unused, double *out_8769711467856925084) {
   out_8769711467856925084[0] = state[4];
}
void H_30(double *state, double *unused, double *out_789221452757328808) {
   out_789221452757328808[0] = 0;
   out_789221452757328808[1] = 0;
   out_789221452757328808[2] = 0;
   out_789221452757328808[3] = 0;
   out_789221452757328808[4] = 1;
   out_789221452757328808[5] = 0;
   out_789221452757328808[6] = 0;
   out_789221452757328808[7] = 0;
   out_789221452757328808[8] = 0;
}
void h_26(double *state, double *unused, double *out_4568584500733172853) {
   out_4568584500733172853[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7049057730138633659) {
   out_7049057730138633659[0] = 0;
   out_7049057730138633659[1] = 0;
   out_7049057730138633659[2] = 0;
   out_7049057730138633659[3] = 0;
   out_7049057730138633659[4] = 0;
   out_7049057730138633659[5] = 0;
   out_7049057730138633659[6] = 0;
   out_7049057730138633659[7] = 1;
   out_7049057730138633659[8] = 0;
}
void h_27(double *state, double *unused, double *out_2359437393968307228) {
   out_2359437393968307228[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1434372618426614409) {
   out_1434372618426614409[0] = 0;
   out_1434372618426614409[1] = 0;
   out_1434372618426614409[2] = 0;
   out_1434372618426614409[3] = 1;
   out_1434372618426614409[4] = 0;
   out_1434372618426614409[5] = 0;
   out_1434372618426614409[6] = 0;
   out_1434372618426614409[7] = 0;
   out_1434372618426614409[8] = 0;
}
void h_29(double *state, double *unused, double *out_4451125390017301585) {
   out_4451125390017301585[0] = state[1];
}
void H_29(double *state, double *unused, double *out_278990108442936624) {
   out_278990108442936624[0] = 0;
   out_278990108442936624[1] = 1;
   out_278990108442936624[2] = 0;
   out_278990108442936624[3] = 0;
   out_278990108442936624[4] = 0;
   out_278990108442936624[5] = 0;
   out_278990108442936624[6] = 0;
   out_278990108442936624[7] = 0;
   out_278990108442936624[8] = 0;
}
void h_28(double *state, double *unused, double *out_6641001235211967736) {
   out_6641001235211967736[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5361389125512467198) {
   out_5361389125512467198[0] = 1;
   out_5361389125512467198[1] = 0;
   out_5361389125512467198[2] = 0;
   out_5361389125512467198[3] = 0;
   out_5361389125512467198[4] = 0;
   out_5361389125512467198[5] = 0;
   out_5361389125512467198[6] = 0;
   out_5361389125512467198[7] = 0;
   out_5361389125512467198[8] = 0;
}
void h_31(double *state, double *unused, double *out_8651704073923548340) {
   out_8651704073923548340[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3276908449387617007) {
   out_3276908449387617007[0] = 0;
   out_3276908449387617007[1] = 0;
   out_3276908449387617007[2] = 0;
   out_3276908449387617007[3] = 0;
   out_3276908449387617007[4] = 0;
   out_3276908449387617007[5] = 0;
   out_3276908449387617007[6] = 0;
   out_3276908449387617007[7] = 0;
   out_3276908449387617007[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_198445823836928788) {
  err_fun(nom_x, delta_x, out_198445823836928788);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7598262341626218705) {
  inv_err_fun(nom_x, true_x, out_7598262341626218705);
}
void car_H_mod_fun(double *state, double *out_2149329292292161718) {
  H_mod_fun(state, out_2149329292292161718);
}
void car_f_fun(double *state, double dt, double *out_7137556374309530724) {
  f_fun(state,  dt, out_7137556374309530724);
}
void car_F_fun(double *state, double dt, double *out_8393538857825963338) {
  F_fun(state,  dt, out_8393538857825963338);
}
void car_h_25(double *state, double *unused, double *out_1913767349031954091) {
  h_25(state, unused, out_1913767349031954091);
}
void car_H_25(double *state, double *unused, double *out_3307554411264577435) {
  H_25(state, unused, out_3307554411264577435);
}
void car_h_24(double *state, double *unused, double *out_1050780426769953054) {
  h_24(state, unused, out_1050780426769953054);
}
void car_H_24(double *state, double *unused, double *out_8054063738231086469) {
  H_24(state, unused, out_8054063738231086469);
}
void car_h_30(double *state, double *unused, double *out_8769711467856925084) {
  h_30(state, unused, out_8769711467856925084);
}
void car_H_30(double *state, double *unused, double *out_789221452757328808) {
  H_30(state, unused, out_789221452757328808);
}
void car_h_26(double *state, double *unused, double *out_4568584500733172853) {
  h_26(state, unused, out_4568584500733172853);
}
void car_H_26(double *state, double *unused, double *out_7049057730138633659) {
  H_26(state, unused, out_7049057730138633659);
}
void car_h_27(double *state, double *unused, double *out_2359437393968307228) {
  h_27(state, unused, out_2359437393968307228);
}
void car_H_27(double *state, double *unused, double *out_1434372618426614409) {
  H_27(state, unused, out_1434372618426614409);
}
void car_h_29(double *state, double *unused, double *out_4451125390017301585) {
  h_29(state, unused, out_4451125390017301585);
}
void car_H_29(double *state, double *unused, double *out_278990108442936624) {
  H_29(state, unused, out_278990108442936624);
}
void car_h_28(double *state, double *unused, double *out_6641001235211967736) {
  h_28(state, unused, out_6641001235211967736);
}
void car_H_28(double *state, double *unused, double *out_5361389125512467198) {
  H_28(state, unused, out_5361389125512467198);
}
void car_h_31(double *state, double *unused, double *out_8651704073923548340) {
  h_31(state, unused, out_8651704073923548340);
}
void car_H_31(double *state, double *unused, double *out_3276908449387617007) {
  H_31(state, unused, out_3276908449387617007);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
