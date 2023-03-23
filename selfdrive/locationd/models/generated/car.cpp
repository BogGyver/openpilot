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
void err_fun(double *nom_x, double *delta_x, double *out_7853339013055020703) {
   out_7853339013055020703[0] = delta_x[0] + nom_x[0];
   out_7853339013055020703[1] = delta_x[1] + nom_x[1];
   out_7853339013055020703[2] = delta_x[2] + nom_x[2];
   out_7853339013055020703[3] = delta_x[3] + nom_x[3];
   out_7853339013055020703[4] = delta_x[4] + nom_x[4];
   out_7853339013055020703[5] = delta_x[5] + nom_x[5];
   out_7853339013055020703[6] = delta_x[6] + nom_x[6];
   out_7853339013055020703[7] = delta_x[7] + nom_x[7];
   out_7853339013055020703[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4568207359509177387) {
   out_4568207359509177387[0] = -nom_x[0] + true_x[0];
   out_4568207359509177387[1] = -nom_x[1] + true_x[1];
   out_4568207359509177387[2] = -nom_x[2] + true_x[2];
   out_4568207359509177387[3] = -nom_x[3] + true_x[3];
   out_4568207359509177387[4] = -nom_x[4] + true_x[4];
   out_4568207359509177387[5] = -nom_x[5] + true_x[5];
   out_4568207359509177387[6] = -nom_x[6] + true_x[6];
   out_4568207359509177387[7] = -nom_x[7] + true_x[7];
   out_4568207359509177387[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_400617013284271733) {
   out_400617013284271733[0] = 1.0;
   out_400617013284271733[1] = 0;
   out_400617013284271733[2] = 0;
   out_400617013284271733[3] = 0;
   out_400617013284271733[4] = 0;
   out_400617013284271733[5] = 0;
   out_400617013284271733[6] = 0;
   out_400617013284271733[7] = 0;
   out_400617013284271733[8] = 0;
   out_400617013284271733[9] = 0;
   out_400617013284271733[10] = 1.0;
   out_400617013284271733[11] = 0;
   out_400617013284271733[12] = 0;
   out_400617013284271733[13] = 0;
   out_400617013284271733[14] = 0;
   out_400617013284271733[15] = 0;
   out_400617013284271733[16] = 0;
   out_400617013284271733[17] = 0;
   out_400617013284271733[18] = 0;
   out_400617013284271733[19] = 0;
   out_400617013284271733[20] = 1.0;
   out_400617013284271733[21] = 0;
   out_400617013284271733[22] = 0;
   out_400617013284271733[23] = 0;
   out_400617013284271733[24] = 0;
   out_400617013284271733[25] = 0;
   out_400617013284271733[26] = 0;
   out_400617013284271733[27] = 0;
   out_400617013284271733[28] = 0;
   out_400617013284271733[29] = 0;
   out_400617013284271733[30] = 1.0;
   out_400617013284271733[31] = 0;
   out_400617013284271733[32] = 0;
   out_400617013284271733[33] = 0;
   out_400617013284271733[34] = 0;
   out_400617013284271733[35] = 0;
   out_400617013284271733[36] = 0;
   out_400617013284271733[37] = 0;
   out_400617013284271733[38] = 0;
   out_400617013284271733[39] = 0;
   out_400617013284271733[40] = 1.0;
   out_400617013284271733[41] = 0;
   out_400617013284271733[42] = 0;
   out_400617013284271733[43] = 0;
   out_400617013284271733[44] = 0;
   out_400617013284271733[45] = 0;
   out_400617013284271733[46] = 0;
   out_400617013284271733[47] = 0;
   out_400617013284271733[48] = 0;
   out_400617013284271733[49] = 0;
   out_400617013284271733[50] = 1.0;
   out_400617013284271733[51] = 0;
   out_400617013284271733[52] = 0;
   out_400617013284271733[53] = 0;
   out_400617013284271733[54] = 0;
   out_400617013284271733[55] = 0;
   out_400617013284271733[56] = 0;
   out_400617013284271733[57] = 0;
   out_400617013284271733[58] = 0;
   out_400617013284271733[59] = 0;
   out_400617013284271733[60] = 1.0;
   out_400617013284271733[61] = 0;
   out_400617013284271733[62] = 0;
   out_400617013284271733[63] = 0;
   out_400617013284271733[64] = 0;
   out_400617013284271733[65] = 0;
   out_400617013284271733[66] = 0;
   out_400617013284271733[67] = 0;
   out_400617013284271733[68] = 0;
   out_400617013284271733[69] = 0;
   out_400617013284271733[70] = 1.0;
   out_400617013284271733[71] = 0;
   out_400617013284271733[72] = 0;
   out_400617013284271733[73] = 0;
   out_400617013284271733[74] = 0;
   out_400617013284271733[75] = 0;
   out_400617013284271733[76] = 0;
   out_400617013284271733[77] = 0;
   out_400617013284271733[78] = 0;
   out_400617013284271733[79] = 0;
   out_400617013284271733[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_536377625142687133) {
   out_536377625142687133[0] = state[0];
   out_536377625142687133[1] = state[1];
   out_536377625142687133[2] = state[2];
   out_536377625142687133[3] = state[3];
   out_536377625142687133[4] = state[4];
   out_536377625142687133[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_536377625142687133[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_536377625142687133[7] = state[7];
   out_536377625142687133[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8637102387366365346) {
   out_8637102387366365346[0] = 1;
   out_8637102387366365346[1] = 0;
   out_8637102387366365346[2] = 0;
   out_8637102387366365346[3] = 0;
   out_8637102387366365346[4] = 0;
   out_8637102387366365346[5] = 0;
   out_8637102387366365346[6] = 0;
   out_8637102387366365346[7] = 0;
   out_8637102387366365346[8] = 0;
   out_8637102387366365346[9] = 0;
   out_8637102387366365346[10] = 1;
   out_8637102387366365346[11] = 0;
   out_8637102387366365346[12] = 0;
   out_8637102387366365346[13] = 0;
   out_8637102387366365346[14] = 0;
   out_8637102387366365346[15] = 0;
   out_8637102387366365346[16] = 0;
   out_8637102387366365346[17] = 0;
   out_8637102387366365346[18] = 0;
   out_8637102387366365346[19] = 0;
   out_8637102387366365346[20] = 1;
   out_8637102387366365346[21] = 0;
   out_8637102387366365346[22] = 0;
   out_8637102387366365346[23] = 0;
   out_8637102387366365346[24] = 0;
   out_8637102387366365346[25] = 0;
   out_8637102387366365346[26] = 0;
   out_8637102387366365346[27] = 0;
   out_8637102387366365346[28] = 0;
   out_8637102387366365346[29] = 0;
   out_8637102387366365346[30] = 1;
   out_8637102387366365346[31] = 0;
   out_8637102387366365346[32] = 0;
   out_8637102387366365346[33] = 0;
   out_8637102387366365346[34] = 0;
   out_8637102387366365346[35] = 0;
   out_8637102387366365346[36] = 0;
   out_8637102387366365346[37] = 0;
   out_8637102387366365346[38] = 0;
   out_8637102387366365346[39] = 0;
   out_8637102387366365346[40] = 1;
   out_8637102387366365346[41] = 0;
   out_8637102387366365346[42] = 0;
   out_8637102387366365346[43] = 0;
   out_8637102387366365346[44] = 0;
   out_8637102387366365346[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8637102387366365346[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8637102387366365346[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8637102387366365346[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8637102387366365346[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8637102387366365346[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8637102387366365346[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8637102387366365346[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8637102387366365346[53] = -9.8000000000000007*dt;
   out_8637102387366365346[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8637102387366365346[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8637102387366365346[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8637102387366365346[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8637102387366365346[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8637102387366365346[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8637102387366365346[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8637102387366365346[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8637102387366365346[62] = 0;
   out_8637102387366365346[63] = 0;
   out_8637102387366365346[64] = 0;
   out_8637102387366365346[65] = 0;
   out_8637102387366365346[66] = 0;
   out_8637102387366365346[67] = 0;
   out_8637102387366365346[68] = 0;
   out_8637102387366365346[69] = 0;
   out_8637102387366365346[70] = 1;
   out_8637102387366365346[71] = 0;
   out_8637102387366365346[72] = 0;
   out_8637102387366365346[73] = 0;
   out_8637102387366365346[74] = 0;
   out_8637102387366365346[75] = 0;
   out_8637102387366365346[76] = 0;
   out_8637102387366365346[77] = 0;
   out_8637102387366365346[78] = 0;
   out_8637102387366365346[79] = 0;
   out_8637102387366365346[80] = 1;
}
void h_25(double *state, double *unused, double *out_3784825874740371840) {
   out_3784825874740371840[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3254223637679582827) {
   out_3254223637679582827[0] = 0;
   out_3254223637679582827[1] = 0;
   out_3254223637679582827[2] = 0;
   out_3254223637679582827[3] = 0;
   out_3254223637679582827[4] = 0;
   out_3254223637679582827[5] = 0;
   out_3254223637679582827[6] = 1;
   out_3254223637679582827[7] = 0;
   out_3254223637679582827[8] = 0;
}
void h_24(double *state, double *unused, double *out_5977641087208495030) {
   out_5977641087208495030[0] = state[4];
   out_5977641087208495030[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3724681119722921551) {
   out_3724681119722921551[0] = 0;
   out_3724681119722921551[1] = 0;
   out_3724681119722921551[2] = 0;
   out_3724681119722921551[3] = 0;
   out_3724681119722921551[4] = 1;
   out_3724681119722921551[5] = 0;
   out_3724681119722921551[6] = 0;
   out_3724681119722921551[7] = 0;
   out_3724681119722921551[8] = 0;
   out_3724681119722921551[9] = 0;
   out_3724681119722921551[10] = 0;
   out_3724681119722921551[11] = 0;
   out_3724681119722921551[12] = 0;
   out_3724681119722921551[13] = 0;
   out_3724681119722921551[14] = 1;
   out_3724681119722921551[15] = 0;
   out_3724681119722921551[16] = 0;
   out_3724681119722921551[17] = 0;
}
void h_30(double *state, double *unused, double *out_7891082972618828840) {
   out_7891082972618828840[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3383562584822822897) {
   out_3383562584822822897[0] = 0;
   out_3383562584822822897[1] = 0;
   out_3383562584822822897[2] = 0;
   out_3383562584822822897[3] = 0;
   out_3383562584822822897[4] = 1;
   out_3383562584822822897[5] = 0;
   out_3383562584822822897[6] = 0;
   out_3383562584822822897[7] = 0;
   out_3383562584822822897[8] = 0;
}
void h_26(double *state, double *unused, double *out_2889409704957983195) {
   out_2889409704957983195[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6995726956553639051) {
   out_6995726956553639051[0] = 0;
   out_6995726956553639051[1] = 0;
   out_6995726956553639051[2] = 0;
   out_6995726956553639051[3] = 0;
   out_6995726956553639051[4] = 0;
   out_6995726956553639051[5] = 0;
   out_6995726956553639051[6] = 0;
   out_6995726956553639051[7] = 1;
   out_6995726956553639051[8] = 0;
}
void h_27(double *state, double *unused, double *out_1216847392920473805) {
   out_1216847392920473805[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5558325896623247808) {
   out_5558325896623247808[0] = 0;
   out_5558325896623247808[1] = 0;
   out_5558325896623247808[2] = 0;
   out_5558325896623247808[3] = 1;
   out_5558325896623247808[4] = 0;
   out_5558325896623247808[5] = 0;
   out_5558325896623247808[6] = 0;
   out_5558325896623247808[7] = 0;
   out_5558325896623247808[8] = 0;
}
void h_29(double *state, double *unused, double *out_5553987833429877131) {
   out_5553987833429877131[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2873331240508430713) {
   out_2873331240508430713[0] = 0;
   out_2873331240508430713[1] = 1;
   out_2873331240508430713[2] = 0;
   out_2873331240508430713[3] = 0;
   out_2873331240508430713[4] = 0;
   out_2873331240508430713[5] = 0;
   out_2873331240508430713[6] = 0;
   out_2873331240508430713[7] = 0;
   out_2873331240508430713[8] = 0;
}
void h_28(double *state, double *unused, double *out_273337540018811370) {
   out_273337540018811370[0] = state[0];
}
void H_28(double *state, double *unused, double *out_909700968943104462) {
   out_909700968943104462[0] = 1;
   out_909700968943104462[1] = 0;
   out_909700968943104462[2] = 0;
   out_909700968943104462[3] = 0;
   out_909700968943104462[4] = 0;
   out_909700968943104462[5] = 0;
   out_909700968943104462[6] = 0;
   out_909700968943104462[7] = 0;
   out_909700968943104462[8] = 0;
}
void h_31(double *state, double *unused, double *out_4108113620144151519) {
   out_4108113620144151519[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3223577675802622399) {
   out_3223577675802622399[0] = 0;
   out_3223577675802622399[1] = 0;
   out_3223577675802622399[2] = 0;
   out_3223577675802622399[3] = 0;
   out_3223577675802622399[4] = 0;
   out_3223577675802622399[5] = 0;
   out_3223577675802622399[6] = 0;
   out_3223577675802622399[7] = 0;
   out_3223577675802622399[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7853339013055020703) {
  err_fun(nom_x, delta_x, out_7853339013055020703);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4568207359509177387) {
  inv_err_fun(nom_x, true_x, out_4568207359509177387);
}
void car_H_mod_fun(double *state, double *out_400617013284271733) {
  H_mod_fun(state, out_400617013284271733);
}
void car_f_fun(double *state, double dt, double *out_536377625142687133) {
  f_fun(state,  dt, out_536377625142687133);
}
void car_F_fun(double *state, double dt, double *out_8637102387366365346) {
  F_fun(state,  dt, out_8637102387366365346);
}
void car_h_25(double *state, double *unused, double *out_3784825874740371840) {
  h_25(state, unused, out_3784825874740371840);
}
void car_H_25(double *state, double *unused, double *out_3254223637679582827) {
  H_25(state, unused, out_3254223637679582827);
}
void car_h_24(double *state, double *unused, double *out_5977641087208495030) {
  h_24(state, unused, out_5977641087208495030);
}
void car_H_24(double *state, double *unused, double *out_3724681119722921551) {
  H_24(state, unused, out_3724681119722921551);
}
void car_h_30(double *state, double *unused, double *out_7891082972618828840) {
  h_30(state, unused, out_7891082972618828840);
}
void car_H_30(double *state, double *unused, double *out_3383562584822822897) {
  H_30(state, unused, out_3383562584822822897);
}
void car_h_26(double *state, double *unused, double *out_2889409704957983195) {
  h_26(state, unused, out_2889409704957983195);
}
void car_H_26(double *state, double *unused, double *out_6995726956553639051) {
  H_26(state, unused, out_6995726956553639051);
}
void car_h_27(double *state, double *unused, double *out_1216847392920473805) {
  h_27(state, unused, out_1216847392920473805);
}
void car_H_27(double *state, double *unused, double *out_5558325896623247808) {
  H_27(state, unused, out_5558325896623247808);
}
void car_h_29(double *state, double *unused, double *out_5553987833429877131) {
  h_29(state, unused, out_5553987833429877131);
}
void car_H_29(double *state, double *unused, double *out_2873331240508430713) {
  H_29(state, unused, out_2873331240508430713);
}
void car_h_28(double *state, double *unused, double *out_273337540018811370) {
  h_28(state, unused, out_273337540018811370);
}
void car_H_28(double *state, double *unused, double *out_909700968943104462) {
  H_28(state, unused, out_909700968943104462);
}
void car_h_31(double *state, double *unused, double *out_4108113620144151519) {
  h_31(state, unused, out_4108113620144151519);
}
void car_H_31(double *state, double *unused, double *out_3223577675802622399) {
  H_31(state, unused, out_3223577675802622399);
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
