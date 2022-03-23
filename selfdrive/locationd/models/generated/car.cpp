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
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1475588756916852268) {
   out_1475588756916852268[0] = delta_x[0] + nom_x[0];
   out_1475588756916852268[1] = delta_x[1] + nom_x[1];
   out_1475588756916852268[2] = delta_x[2] + nom_x[2];
   out_1475588756916852268[3] = delta_x[3] + nom_x[3];
   out_1475588756916852268[4] = delta_x[4] + nom_x[4];
   out_1475588756916852268[5] = delta_x[5] + nom_x[5];
   out_1475588756916852268[6] = delta_x[6] + nom_x[6];
   out_1475588756916852268[7] = delta_x[7] + nom_x[7];
   out_1475588756916852268[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3596983127668324109) {
   out_3596983127668324109[0] = -nom_x[0] + true_x[0];
   out_3596983127668324109[1] = -nom_x[1] + true_x[1];
   out_3596983127668324109[2] = -nom_x[2] + true_x[2];
   out_3596983127668324109[3] = -nom_x[3] + true_x[3];
   out_3596983127668324109[4] = -nom_x[4] + true_x[4];
   out_3596983127668324109[5] = -nom_x[5] + true_x[5];
   out_3596983127668324109[6] = -nom_x[6] + true_x[6];
   out_3596983127668324109[7] = -nom_x[7] + true_x[7];
   out_3596983127668324109[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3389099407762428077) {
   out_3389099407762428077[0] = 1.0;
   out_3389099407762428077[1] = 0;
   out_3389099407762428077[2] = 0;
   out_3389099407762428077[3] = 0;
   out_3389099407762428077[4] = 0;
   out_3389099407762428077[5] = 0;
   out_3389099407762428077[6] = 0;
   out_3389099407762428077[7] = 0;
   out_3389099407762428077[8] = 0;
   out_3389099407762428077[9] = 0;
   out_3389099407762428077[10] = 1.0;
   out_3389099407762428077[11] = 0;
   out_3389099407762428077[12] = 0;
   out_3389099407762428077[13] = 0;
   out_3389099407762428077[14] = 0;
   out_3389099407762428077[15] = 0;
   out_3389099407762428077[16] = 0;
   out_3389099407762428077[17] = 0;
   out_3389099407762428077[18] = 0;
   out_3389099407762428077[19] = 0;
   out_3389099407762428077[20] = 1.0;
   out_3389099407762428077[21] = 0;
   out_3389099407762428077[22] = 0;
   out_3389099407762428077[23] = 0;
   out_3389099407762428077[24] = 0;
   out_3389099407762428077[25] = 0;
   out_3389099407762428077[26] = 0;
   out_3389099407762428077[27] = 0;
   out_3389099407762428077[28] = 0;
   out_3389099407762428077[29] = 0;
   out_3389099407762428077[30] = 1.0;
   out_3389099407762428077[31] = 0;
   out_3389099407762428077[32] = 0;
   out_3389099407762428077[33] = 0;
   out_3389099407762428077[34] = 0;
   out_3389099407762428077[35] = 0;
   out_3389099407762428077[36] = 0;
   out_3389099407762428077[37] = 0;
   out_3389099407762428077[38] = 0;
   out_3389099407762428077[39] = 0;
   out_3389099407762428077[40] = 1.0;
   out_3389099407762428077[41] = 0;
   out_3389099407762428077[42] = 0;
   out_3389099407762428077[43] = 0;
   out_3389099407762428077[44] = 0;
   out_3389099407762428077[45] = 0;
   out_3389099407762428077[46] = 0;
   out_3389099407762428077[47] = 0;
   out_3389099407762428077[48] = 0;
   out_3389099407762428077[49] = 0;
   out_3389099407762428077[50] = 1.0;
   out_3389099407762428077[51] = 0;
   out_3389099407762428077[52] = 0;
   out_3389099407762428077[53] = 0;
   out_3389099407762428077[54] = 0;
   out_3389099407762428077[55] = 0;
   out_3389099407762428077[56] = 0;
   out_3389099407762428077[57] = 0;
   out_3389099407762428077[58] = 0;
   out_3389099407762428077[59] = 0;
   out_3389099407762428077[60] = 1.0;
   out_3389099407762428077[61] = 0;
   out_3389099407762428077[62] = 0;
   out_3389099407762428077[63] = 0;
   out_3389099407762428077[64] = 0;
   out_3389099407762428077[65] = 0;
   out_3389099407762428077[66] = 0;
   out_3389099407762428077[67] = 0;
   out_3389099407762428077[68] = 0;
   out_3389099407762428077[69] = 0;
   out_3389099407762428077[70] = 1.0;
   out_3389099407762428077[71] = 0;
   out_3389099407762428077[72] = 0;
   out_3389099407762428077[73] = 0;
   out_3389099407762428077[74] = 0;
   out_3389099407762428077[75] = 0;
   out_3389099407762428077[76] = 0;
   out_3389099407762428077[77] = 0;
   out_3389099407762428077[78] = 0;
   out_3389099407762428077[79] = 0;
   out_3389099407762428077[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5045286729280014390) {
   out_5045286729280014390[0] = state[0];
   out_5045286729280014390[1] = state[1];
   out_5045286729280014390[2] = state[2];
   out_5045286729280014390[3] = state[3];
   out_5045286729280014390[4] = state[4];
   out_5045286729280014390[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5045286729280014390[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5045286729280014390[7] = state[7];
   out_5045286729280014390[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8901810177871386405) {
   out_8901810177871386405[0] = 1;
   out_8901810177871386405[1] = 0;
   out_8901810177871386405[2] = 0;
   out_8901810177871386405[3] = 0;
   out_8901810177871386405[4] = 0;
   out_8901810177871386405[5] = 0;
   out_8901810177871386405[6] = 0;
   out_8901810177871386405[7] = 0;
   out_8901810177871386405[8] = 0;
   out_8901810177871386405[9] = 0;
   out_8901810177871386405[10] = 1;
   out_8901810177871386405[11] = 0;
   out_8901810177871386405[12] = 0;
   out_8901810177871386405[13] = 0;
   out_8901810177871386405[14] = 0;
   out_8901810177871386405[15] = 0;
   out_8901810177871386405[16] = 0;
   out_8901810177871386405[17] = 0;
   out_8901810177871386405[18] = 0;
   out_8901810177871386405[19] = 0;
   out_8901810177871386405[20] = 1;
   out_8901810177871386405[21] = 0;
   out_8901810177871386405[22] = 0;
   out_8901810177871386405[23] = 0;
   out_8901810177871386405[24] = 0;
   out_8901810177871386405[25] = 0;
   out_8901810177871386405[26] = 0;
   out_8901810177871386405[27] = 0;
   out_8901810177871386405[28] = 0;
   out_8901810177871386405[29] = 0;
   out_8901810177871386405[30] = 1;
   out_8901810177871386405[31] = 0;
   out_8901810177871386405[32] = 0;
   out_8901810177871386405[33] = 0;
   out_8901810177871386405[34] = 0;
   out_8901810177871386405[35] = 0;
   out_8901810177871386405[36] = 0;
   out_8901810177871386405[37] = 0;
   out_8901810177871386405[38] = 0;
   out_8901810177871386405[39] = 0;
   out_8901810177871386405[40] = 1;
   out_8901810177871386405[41] = 0;
   out_8901810177871386405[42] = 0;
   out_8901810177871386405[43] = 0;
   out_8901810177871386405[44] = 0;
   out_8901810177871386405[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8901810177871386405[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8901810177871386405[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8901810177871386405[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8901810177871386405[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8901810177871386405[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8901810177871386405[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8901810177871386405[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8901810177871386405[53] = -9.8000000000000007*dt;
   out_8901810177871386405[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8901810177871386405[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8901810177871386405[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8901810177871386405[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8901810177871386405[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8901810177871386405[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8901810177871386405[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8901810177871386405[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8901810177871386405[62] = 0;
   out_8901810177871386405[63] = 0;
   out_8901810177871386405[64] = 0;
   out_8901810177871386405[65] = 0;
   out_8901810177871386405[66] = 0;
   out_8901810177871386405[67] = 0;
   out_8901810177871386405[68] = 0;
   out_8901810177871386405[69] = 0;
   out_8901810177871386405[70] = 1;
   out_8901810177871386405[71] = 0;
   out_8901810177871386405[72] = 0;
   out_8901810177871386405[73] = 0;
   out_8901810177871386405[74] = 0;
   out_8901810177871386405[75] = 0;
   out_8901810177871386405[76] = 0;
   out_8901810177871386405[77] = 0;
   out_8901810177871386405[78] = 0;
   out_8901810177871386405[79] = 0;
   out_8901810177871386405[80] = 1;
}
void h_25(double *state, double *unused, double *out_2445776787221609462) {
   out_2445776787221609462[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5198710936402375667) {
   out_5198710936402375667[0] = 0;
   out_5198710936402375667[1] = 0;
   out_5198710936402375667[2] = 0;
   out_5198710936402375667[3] = 0;
   out_5198710936402375667[4] = 0;
   out_5198710936402375667[5] = 0;
   out_5198710936402375667[6] = 1;
   out_5198710936402375667[7] = 0;
   out_5198710936402375667[8] = 0;
}
void h_24(double *state, double *unused, double *out_4307300272860009815) {
   out_4307300272860009815[0] = state[4];
   out_4307300272860009815[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8379218272279469097) {
   out_8379218272279469097[0] = 0;
   out_8379218272279469097[1] = 0;
   out_8379218272279469097[2] = 0;
   out_8379218272279469097[3] = 0;
   out_8379218272279469097[4] = 1;
   out_8379218272279469097[5] = 0;
   out_8379218272279469097[6] = 0;
   out_8379218272279469097[7] = 0;
   out_8379218272279469097[8] = 0;
   out_8379218272279469097[9] = 0;
   out_8379218272279469097[10] = 0;
   out_8379218272279469097[11] = 0;
   out_8379218272279469097[12] = 0;
   out_8379218272279469097[13] = 0;
   out_8379218272279469097[14] = 1;
   out_8379218272279469097[15] = 0;
   out_8379218272279469097[16] = 0;
   out_8379218272279469097[17] = 0;
}
void h_30(double *state, double *unused, double *out_5317488469599062979) {
   out_5317488469599062979[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8720336807179567751) {
   out_8720336807179567751[0] = 0;
   out_8720336807179567751[1] = 0;
   out_8720336807179567751[2] = 0;
   out_8720336807179567751[3] = 0;
   out_8720336807179567751[4] = 1;
   out_8720336807179567751[5] = 0;
   out_8720336807179567751[6] = 0;
   out_8720336807179567751[7] = 0;
   out_8720336807179567751[8] = 0;
}
void h_26(double *state, double *unused, double *out_7250970116551773549) {
   out_7250970116551773549[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8940214255276431891) {
   out_8940214255276431891[0] = 0;
   out_8940214255276431891[1] = 0;
   out_8940214255276431891[2] = 0;
   out_8940214255276431891[3] = 0;
   out_8940214255276431891[4] = 0;
   out_8940214255276431891[5] = 0;
   out_8940214255276431891[6] = 0;
   out_8940214255276431891[7] = 1;
   out_8940214255276431891[8] = 0;
}
void h_27(double *state, double *unused, double *out_970496502427460554) {
   out_970496502427460554[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6545573495379142840) {
   out_6545573495379142840[0] = 0;
   out_6545573495379142840[1] = 0;
   out_6545573495379142840[2] = 0;
   out_6545573495379142840[3] = 1;
   out_6545573495379142840[4] = 0;
   out_6545573495379142840[5] = 0;
   out_6545573495379142840[6] = 0;
   out_6545573495379142840[7] = 0;
   out_6545573495379142840[8] = 0;
}
void h_29(double *state, double *unused, double *out_315815201938217334) {
   out_315815201938217334[0] = state[1];
}
void H_29(double *state, double *unused, double *out_9216175922215591681) {
   out_9216175922215591681[0] = 0;
   out_9216175922215591681[1] = 1;
   out_9216175922215591681[2] = 0;
   out_9216175922215591681[3] = 0;
   out_9216175922215591681[4] = 0;
   out_9216175922215591681[5] = 0;
   out_9216175922215591681[6] = 0;
   out_9216175922215591681[7] = 0;
   out_9216175922215591681[8] = 0;
}
void h_28(double *state, double *unused, double *out_4031176765233385091) {
   out_4031176765233385091[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7252545650650265430) {
   out_7252545650650265430[0] = 1;
   out_7252545650650265430[1] = 0;
   out_7252545650650265430[2] = 0;
   out_7252545650650265430[3] = 0;
   out_7252545650650265430[4] = 0;
   out_7252545650650265430[5] = 0;
   out_7252545650650265430[6] = 0;
   out_7252545650650265430[7] = 0;
   out_7252545650650265430[8] = 0;
}
void h_31(double *state, double *unused, double *out_5435495863532439723) {
   out_5435495863532439723[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8880321716199768249) {
   out_8880321716199768249[0] = 0;
   out_8880321716199768249[1] = 0;
   out_8880321716199768249[2] = 0;
   out_8880321716199768249[3] = 0;
   out_8880321716199768249[4] = 0;
   out_8880321716199768249[5] = 0;
   out_8880321716199768249[6] = 0;
   out_8880321716199768249[7] = 0;
   out_8880321716199768249[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1475588756916852268) {
  err_fun(nom_x, delta_x, out_1475588756916852268);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3596983127668324109) {
  inv_err_fun(nom_x, true_x, out_3596983127668324109);
}
void car_H_mod_fun(double *state, double *out_3389099407762428077) {
  H_mod_fun(state, out_3389099407762428077);
}
void car_f_fun(double *state, double dt, double *out_5045286729280014390) {
  f_fun(state,  dt, out_5045286729280014390);
}
void car_F_fun(double *state, double dt, double *out_8901810177871386405) {
  F_fun(state,  dt, out_8901810177871386405);
}
void car_h_25(double *state, double *unused, double *out_2445776787221609462) {
  h_25(state, unused, out_2445776787221609462);
}
void car_H_25(double *state, double *unused, double *out_5198710936402375667) {
  H_25(state, unused, out_5198710936402375667);
}
void car_h_24(double *state, double *unused, double *out_4307300272860009815) {
  h_24(state, unused, out_4307300272860009815);
}
void car_H_24(double *state, double *unused, double *out_8379218272279469097) {
  H_24(state, unused, out_8379218272279469097);
}
void car_h_30(double *state, double *unused, double *out_5317488469599062979) {
  h_30(state, unused, out_5317488469599062979);
}
void car_H_30(double *state, double *unused, double *out_8720336807179567751) {
  H_30(state, unused, out_8720336807179567751);
}
void car_h_26(double *state, double *unused, double *out_7250970116551773549) {
  h_26(state, unused, out_7250970116551773549);
}
void car_H_26(double *state, double *unused, double *out_8940214255276431891) {
  H_26(state, unused, out_8940214255276431891);
}
void car_h_27(double *state, double *unused, double *out_970496502427460554) {
  h_27(state, unused, out_970496502427460554);
}
void car_H_27(double *state, double *unused, double *out_6545573495379142840) {
  H_27(state, unused, out_6545573495379142840);
}
void car_h_29(double *state, double *unused, double *out_315815201938217334) {
  h_29(state, unused, out_315815201938217334);
}
void car_H_29(double *state, double *unused, double *out_9216175922215591681) {
  H_29(state, unused, out_9216175922215591681);
}
void car_h_28(double *state, double *unused, double *out_4031176765233385091) {
  h_28(state, unused, out_4031176765233385091);
}
void car_H_28(double *state, double *unused, double *out_7252545650650265430) {
  H_28(state, unused, out_7252545650650265430);
}
void car_h_31(double *state, double *unused, double *out_5435495863532439723) {
  h_31(state, unused, out_5435495863532439723);
}
void car_H_31(double *state, double *unused, double *out_8880321716199768249) {
  H_31(state, unused, out_8880321716199768249);
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

ekf_init(car);
