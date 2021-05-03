#include "car.h"

namespace {
#define DIM 8
#define EDIM 8
#define MEDIM 8
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
const static double MAHA_THRESH_28 = 5.991464547107981;

/******************************************************************************
 *                      Code generated with sympy 1.7.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_283129011095924522) {
   out_283129011095924522[0] = delta_x[0] + nom_x[0];
   out_283129011095924522[1] = delta_x[1] + nom_x[1];
   out_283129011095924522[2] = delta_x[2] + nom_x[2];
   out_283129011095924522[3] = delta_x[3] + nom_x[3];
   out_283129011095924522[4] = delta_x[4] + nom_x[4];
   out_283129011095924522[5] = delta_x[5] + nom_x[5];
   out_283129011095924522[6] = delta_x[6] + nom_x[6];
   out_283129011095924522[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4497310152589916621) {
   out_4497310152589916621[0] = -nom_x[0] + true_x[0];
   out_4497310152589916621[1] = -nom_x[1] + true_x[1];
   out_4497310152589916621[2] = -nom_x[2] + true_x[2];
   out_4497310152589916621[3] = -nom_x[3] + true_x[3];
   out_4497310152589916621[4] = -nom_x[4] + true_x[4];
   out_4497310152589916621[5] = -nom_x[5] + true_x[5];
   out_4497310152589916621[6] = -nom_x[6] + true_x[6];
   out_4497310152589916621[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_1841431657313326858) {
   out_1841431657313326858[0] = 1.0;
   out_1841431657313326858[1] = 0.0;
   out_1841431657313326858[2] = 0.0;
   out_1841431657313326858[3] = 0.0;
   out_1841431657313326858[4] = 0.0;
   out_1841431657313326858[5] = 0.0;
   out_1841431657313326858[6] = 0.0;
   out_1841431657313326858[7] = 0.0;
   out_1841431657313326858[8] = 0.0;
   out_1841431657313326858[9] = 1.0;
   out_1841431657313326858[10] = 0.0;
   out_1841431657313326858[11] = 0.0;
   out_1841431657313326858[12] = 0.0;
   out_1841431657313326858[13] = 0.0;
   out_1841431657313326858[14] = 0.0;
   out_1841431657313326858[15] = 0.0;
   out_1841431657313326858[16] = 0.0;
   out_1841431657313326858[17] = 0.0;
   out_1841431657313326858[18] = 1.0;
   out_1841431657313326858[19] = 0.0;
   out_1841431657313326858[20] = 0.0;
   out_1841431657313326858[21] = 0.0;
   out_1841431657313326858[22] = 0.0;
   out_1841431657313326858[23] = 0.0;
   out_1841431657313326858[24] = 0.0;
   out_1841431657313326858[25] = 0.0;
   out_1841431657313326858[26] = 0.0;
   out_1841431657313326858[27] = 1.0;
   out_1841431657313326858[28] = 0.0;
   out_1841431657313326858[29] = 0.0;
   out_1841431657313326858[30] = 0.0;
   out_1841431657313326858[31] = 0.0;
   out_1841431657313326858[32] = 0.0;
   out_1841431657313326858[33] = 0.0;
   out_1841431657313326858[34] = 0.0;
   out_1841431657313326858[35] = 0.0;
   out_1841431657313326858[36] = 1.0;
   out_1841431657313326858[37] = 0.0;
   out_1841431657313326858[38] = 0.0;
   out_1841431657313326858[39] = 0.0;
   out_1841431657313326858[40] = 0.0;
   out_1841431657313326858[41] = 0.0;
   out_1841431657313326858[42] = 0.0;
   out_1841431657313326858[43] = 0.0;
   out_1841431657313326858[44] = 0.0;
   out_1841431657313326858[45] = 1.0;
   out_1841431657313326858[46] = 0.0;
   out_1841431657313326858[47] = 0.0;
   out_1841431657313326858[48] = 0.0;
   out_1841431657313326858[49] = 0.0;
   out_1841431657313326858[50] = 0.0;
   out_1841431657313326858[51] = 0.0;
   out_1841431657313326858[52] = 0.0;
   out_1841431657313326858[53] = 0.0;
   out_1841431657313326858[54] = 1.0;
   out_1841431657313326858[55] = 0.0;
   out_1841431657313326858[56] = 0.0;
   out_1841431657313326858[57] = 0.0;
   out_1841431657313326858[58] = 0.0;
   out_1841431657313326858[59] = 0.0;
   out_1841431657313326858[60] = 0.0;
   out_1841431657313326858[61] = 0.0;
   out_1841431657313326858[62] = 0.0;
   out_1841431657313326858[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_7063787066656637565) {
   out_7063787066656637565[0] = state[0];
   out_7063787066656637565[1] = state[1];
   out_7063787066656637565[2] = state[2];
   out_7063787066656637565[3] = state[3];
   out_7063787066656637565[4] = state[4];
   out_7063787066656637565[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7063787066656637565[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7063787066656637565[7] = state[7];
}
void F_fun(double *state, double dt, double *out_7660651947569880398) {
   out_7660651947569880398[0] = 1;
   out_7660651947569880398[1] = 0;
   out_7660651947569880398[2] = 0;
   out_7660651947569880398[3] = 0;
   out_7660651947569880398[4] = 0;
   out_7660651947569880398[5] = 0;
   out_7660651947569880398[6] = 0;
   out_7660651947569880398[7] = 0;
   out_7660651947569880398[8] = 0;
   out_7660651947569880398[9] = 1;
   out_7660651947569880398[10] = 0;
   out_7660651947569880398[11] = 0;
   out_7660651947569880398[12] = 0;
   out_7660651947569880398[13] = 0;
   out_7660651947569880398[14] = 0;
   out_7660651947569880398[15] = 0;
   out_7660651947569880398[16] = 0;
   out_7660651947569880398[17] = 0;
   out_7660651947569880398[18] = 1;
   out_7660651947569880398[19] = 0;
   out_7660651947569880398[20] = 0;
   out_7660651947569880398[21] = 0;
   out_7660651947569880398[22] = 0;
   out_7660651947569880398[23] = 0;
   out_7660651947569880398[24] = 0;
   out_7660651947569880398[25] = 0;
   out_7660651947569880398[26] = 0;
   out_7660651947569880398[27] = 1;
   out_7660651947569880398[28] = 0;
   out_7660651947569880398[29] = 0;
   out_7660651947569880398[30] = 0;
   out_7660651947569880398[31] = 0;
   out_7660651947569880398[32] = 0;
   out_7660651947569880398[33] = 0;
   out_7660651947569880398[34] = 0;
   out_7660651947569880398[35] = 0;
   out_7660651947569880398[36] = 1;
   out_7660651947569880398[37] = 0;
   out_7660651947569880398[38] = 0;
   out_7660651947569880398[39] = 0;
   out_7660651947569880398[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7660651947569880398[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7660651947569880398[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7660651947569880398[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7660651947569880398[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7660651947569880398[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7660651947569880398[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7660651947569880398[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7660651947569880398[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7660651947569880398[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7660651947569880398[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7660651947569880398[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7660651947569880398[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7660651947569880398[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7660651947569880398[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7660651947569880398[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7660651947569880398[56] = 0;
   out_7660651947569880398[57] = 0;
   out_7660651947569880398[58] = 0;
   out_7660651947569880398[59] = 0;
   out_7660651947569880398[60] = 0;
   out_7660651947569880398[61] = 0;
   out_7660651947569880398[62] = 0;
   out_7660651947569880398[63] = 1;
}
void h_25(double *state, double *unused, double *out_5420055961677953059) {
   out_5420055961677953059[0] = state[6];
}
void H_25(double *state, double *unused, double *out_864586791604080428) {
   out_864586791604080428[0] = 0;
   out_864586791604080428[1] = 0;
   out_864586791604080428[2] = 0;
   out_864586791604080428[3] = 0;
   out_864586791604080428[4] = 0;
   out_864586791604080428[5] = 0;
   out_864586791604080428[6] = 1;
   out_864586791604080428[7] = 0;
}
void h_24(double *state, double *unused, double *out_2065971680471068193) {
   out_2065971680471068193[0] = state[4];
   out_2065971680471068193[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5332772925216162998) {
   out_5332772925216162998[0] = 0;
   out_5332772925216162998[1] = 0;
   out_5332772925216162998[2] = 0;
   out_5332772925216162998[3] = 0;
   out_5332772925216162998[4] = 1;
   out_5332772925216162998[5] = 0;
   out_5332772925216162998[6] = 0;
   out_5332772925216162998[7] = 0;
   out_5332772925216162998[8] = 0;
   out_5332772925216162998[9] = 0;
   out_5332772925216162998[10] = 0;
   out_5332772925216162998[11] = 0;
   out_5332772925216162998[12] = 0;
   out_5332772925216162998[13] = 1;
   out_5332772925216162998[14] = 0;
   out_5332772925216162998[15] = 0;
}
void h_30(double *state, double *unused, double *out_7500630675480575444) {
   out_7500630675480575444[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7968258371156287908) {
   out_7968258371156287908[0] = 0;
   out_7968258371156287908[1] = 0;
   out_7968258371156287908[2] = 0;
   out_7968258371156287908[3] = 0;
   out_7968258371156287908[4] = 1;
   out_7968258371156287908[5] = 0;
   out_7968258371156287908[6] = 0;
   out_7968258371156287908[7] = 0;
}
void h_26(double *state, double *unused, double *out_1254179618020401976) {
   out_1254179618020401976[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4840006363335128845) {
   out_4840006363335128845[0] = 0;
   out_4840006363335128845[1] = 0;
   out_4840006363335128845[2] = 0;
   out_4840006363335128845[3] = 0;
   out_4840006363335128845[4] = 0;
   out_4840006363335128845[5] = 0;
   out_4840006363335128845[6] = 0;
   out_4840006363335128845[7] = 1;
}
void h_27(double *state, double *unused, double *out_5865726006614306196) {
   out_5865726006614306196[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6680676383319662596) {
   out_6680676383319662596[0] = 0;
   out_6680676383319662596[1] = 0;
   out_6680676383319662596[2] = 0;
   out_6680676383319662596[3] = 1;
   out_6680676383319662596[4] = 0;
   out_6680676383319662596[5] = 0;
   out_6680676383319662596[6] = 0;
   out_6680676383319662596[7] = 0;
}
void h_29(double *state, double *unused, double *out_4896225354671877793) {
   out_4896225354671877793[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4144459602027455955) {
   out_4144459602027455955[0] = 0;
   out_4144459602027455955[1] = 1;
   out_4144459602027455955[2] = 0;
   out_4144459602027455955[3] = 0;
   out_4144459602027455955[4] = 0;
   out_4144459602027455955[5] = 0;
   out_4144459602027455955[6] = 0;
   out_4144459602027455955[7] = 0;
}
void h_28(double *state, double *unused, double *out_8461947836500736621) {
   out_8461947836500736621[0] = state[5];
   out_8461947836500736621[1] = state[6];
}
void H_28(double *state, double *unused, double *out_1685059553020331868) {
   out_1685059553020331868[0] = 0;
   out_1685059553020331868[1] = 0;
   out_1685059553020331868[2] = 0;
   out_1685059553020331868[3] = 0;
   out_1685059553020331868[4] = 0;
   out_1685059553020331868[5] = 1;
   out_1685059553020331868[6] = 0;
   out_1685059553020331868[7] = 0;
   out_1685059553020331868[8] = 0;
   out_1685059553020331868[9] = 0;
   out_1685059553020331868[10] = 0;
   out_1685059553020331868[11] = 0;
   out_1685059553020331868[12] = 0;
   out_1685059553020331868[13] = 0;
   out_1685059553020331868[14] = 1;
   out_1685059553020331868[15] = 0;
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
  update<2, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_283129011095924522) {
  err_fun(nom_x, delta_x, out_283129011095924522);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4497310152589916621) {
  inv_err_fun(nom_x, true_x, out_4497310152589916621);
}
void car_H_mod_fun(double *state, double *out_1841431657313326858) {
  H_mod_fun(state, out_1841431657313326858);
}
void car_f_fun(double *state, double dt, double *out_7063787066656637565) {
  f_fun(state,  dt, out_7063787066656637565);
}
void car_F_fun(double *state, double dt, double *out_7660651947569880398) {
  F_fun(state,  dt, out_7660651947569880398);
}
void car_h_25(double *state, double *unused, double *out_5420055961677953059) {
  h_25(state, unused, out_5420055961677953059);
}
void car_H_25(double *state, double *unused, double *out_864586791604080428) {
  H_25(state, unused, out_864586791604080428);
}
void car_h_24(double *state, double *unused, double *out_2065971680471068193) {
  h_24(state, unused, out_2065971680471068193);
}
void car_H_24(double *state, double *unused, double *out_5332772925216162998) {
  H_24(state, unused, out_5332772925216162998);
}
void car_h_30(double *state, double *unused, double *out_7500630675480575444) {
  h_30(state, unused, out_7500630675480575444);
}
void car_H_30(double *state, double *unused, double *out_7968258371156287908) {
  H_30(state, unused, out_7968258371156287908);
}
void car_h_26(double *state, double *unused, double *out_1254179618020401976) {
  h_26(state, unused, out_1254179618020401976);
}
void car_H_26(double *state, double *unused, double *out_4840006363335128845) {
  H_26(state, unused, out_4840006363335128845);
}
void car_h_27(double *state, double *unused, double *out_5865726006614306196) {
  h_27(state, unused, out_5865726006614306196);
}
void car_H_27(double *state, double *unused, double *out_6680676383319662596) {
  H_27(state, unused, out_6680676383319662596);
}
void car_h_29(double *state, double *unused, double *out_4896225354671877793) {
  h_29(state, unused, out_4896225354671877793);
}
void car_H_29(double *state, double *unused, double *out_4144459602027455955) {
  H_29(state, unused, out_4144459602027455955);
}
void car_h_28(double *state, double *unused, double *out_8461947836500736621) {
  h_28(state, unused, out_8461947836500736621);
}
void car_H_28(double *state, double *unused, double *out_1685059553020331868) {
  H_28(state, unused, out_1685059553020331868);
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
  .kinds = { 25, 24, 30, 26, 27, 29, 28 },
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
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
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
