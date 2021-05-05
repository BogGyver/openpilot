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
void err_fun(double *nom_x, double *delta_x, double *out_6769788503972096709) {
   out_6769788503972096709[0] = delta_x[0] + nom_x[0];
   out_6769788503972096709[1] = delta_x[1] + nom_x[1];
   out_6769788503972096709[2] = delta_x[2] + nom_x[2];
   out_6769788503972096709[3] = delta_x[3] + nom_x[3];
   out_6769788503972096709[4] = delta_x[4] + nom_x[4];
   out_6769788503972096709[5] = delta_x[5] + nom_x[5];
   out_6769788503972096709[6] = delta_x[6] + nom_x[6];
   out_6769788503972096709[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_152269297933425103) {
   out_152269297933425103[0] = -nom_x[0] + true_x[0];
   out_152269297933425103[1] = -nom_x[1] + true_x[1];
   out_152269297933425103[2] = -nom_x[2] + true_x[2];
   out_152269297933425103[3] = -nom_x[3] + true_x[3];
   out_152269297933425103[4] = -nom_x[4] + true_x[4];
   out_152269297933425103[5] = -nom_x[5] + true_x[5];
   out_152269297933425103[6] = -nom_x[6] + true_x[6];
   out_152269297933425103[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_4881300373756157474) {
   out_4881300373756157474[0] = 1.0;
   out_4881300373756157474[1] = 0.0;
   out_4881300373756157474[2] = 0.0;
   out_4881300373756157474[3] = 0.0;
   out_4881300373756157474[4] = 0.0;
   out_4881300373756157474[5] = 0.0;
   out_4881300373756157474[6] = 0.0;
   out_4881300373756157474[7] = 0.0;
   out_4881300373756157474[8] = 0.0;
   out_4881300373756157474[9] = 1.0;
   out_4881300373756157474[10] = 0.0;
   out_4881300373756157474[11] = 0.0;
   out_4881300373756157474[12] = 0.0;
   out_4881300373756157474[13] = 0.0;
   out_4881300373756157474[14] = 0.0;
   out_4881300373756157474[15] = 0.0;
   out_4881300373756157474[16] = 0.0;
   out_4881300373756157474[17] = 0.0;
   out_4881300373756157474[18] = 1.0;
   out_4881300373756157474[19] = 0.0;
   out_4881300373756157474[20] = 0.0;
   out_4881300373756157474[21] = 0.0;
   out_4881300373756157474[22] = 0.0;
   out_4881300373756157474[23] = 0.0;
   out_4881300373756157474[24] = 0.0;
   out_4881300373756157474[25] = 0.0;
   out_4881300373756157474[26] = 0.0;
   out_4881300373756157474[27] = 1.0;
   out_4881300373756157474[28] = 0.0;
   out_4881300373756157474[29] = 0.0;
   out_4881300373756157474[30] = 0.0;
   out_4881300373756157474[31] = 0.0;
   out_4881300373756157474[32] = 0.0;
   out_4881300373756157474[33] = 0.0;
   out_4881300373756157474[34] = 0.0;
   out_4881300373756157474[35] = 0.0;
   out_4881300373756157474[36] = 1.0;
   out_4881300373756157474[37] = 0.0;
   out_4881300373756157474[38] = 0.0;
   out_4881300373756157474[39] = 0.0;
   out_4881300373756157474[40] = 0.0;
   out_4881300373756157474[41] = 0.0;
   out_4881300373756157474[42] = 0.0;
   out_4881300373756157474[43] = 0.0;
   out_4881300373756157474[44] = 0.0;
   out_4881300373756157474[45] = 1.0;
   out_4881300373756157474[46] = 0.0;
   out_4881300373756157474[47] = 0.0;
   out_4881300373756157474[48] = 0.0;
   out_4881300373756157474[49] = 0.0;
   out_4881300373756157474[50] = 0.0;
   out_4881300373756157474[51] = 0.0;
   out_4881300373756157474[52] = 0.0;
   out_4881300373756157474[53] = 0.0;
   out_4881300373756157474[54] = 1.0;
   out_4881300373756157474[55] = 0.0;
   out_4881300373756157474[56] = 0.0;
   out_4881300373756157474[57] = 0.0;
   out_4881300373756157474[58] = 0.0;
   out_4881300373756157474[59] = 0.0;
   out_4881300373756157474[60] = 0.0;
   out_4881300373756157474[61] = 0.0;
   out_4881300373756157474[62] = 0.0;
   out_4881300373756157474[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_6919783745829379099) {
   out_6919783745829379099[0] = state[0];
   out_6919783745829379099[1] = state[1];
   out_6919783745829379099[2] = state[2];
   out_6919783745829379099[3] = state[3];
   out_6919783745829379099[4] = state[4];
   out_6919783745829379099[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6919783745829379099[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6919783745829379099[7] = state[7];
}
void F_fun(double *state, double dt, double *out_7112290649774386037) {
   out_7112290649774386037[0] = 1;
   out_7112290649774386037[1] = 0;
   out_7112290649774386037[2] = 0;
   out_7112290649774386037[3] = 0;
   out_7112290649774386037[4] = 0;
   out_7112290649774386037[5] = 0;
   out_7112290649774386037[6] = 0;
   out_7112290649774386037[7] = 0;
   out_7112290649774386037[8] = 0;
   out_7112290649774386037[9] = 1;
   out_7112290649774386037[10] = 0;
   out_7112290649774386037[11] = 0;
   out_7112290649774386037[12] = 0;
   out_7112290649774386037[13] = 0;
   out_7112290649774386037[14] = 0;
   out_7112290649774386037[15] = 0;
   out_7112290649774386037[16] = 0;
   out_7112290649774386037[17] = 0;
   out_7112290649774386037[18] = 1;
   out_7112290649774386037[19] = 0;
   out_7112290649774386037[20] = 0;
   out_7112290649774386037[21] = 0;
   out_7112290649774386037[22] = 0;
   out_7112290649774386037[23] = 0;
   out_7112290649774386037[24] = 0;
   out_7112290649774386037[25] = 0;
   out_7112290649774386037[26] = 0;
   out_7112290649774386037[27] = 1;
   out_7112290649774386037[28] = 0;
   out_7112290649774386037[29] = 0;
   out_7112290649774386037[30] = 0;
   out_7112290649774386037[31] = 0;
   out_7112290649774386037[32] = 0;
   out_7112290649774386037[33] = 0;
   out_7112290649774386037[34] = 0;
   out_7112290649774386037[35] = 0;
   out_7112290649774386037[36] = 1;
   out_7112290649774386037[37] = 0;
   out_7112290649774386037[38] = 0;
   out_7112290649774386037[39] = 0;
   out_7112290649774386037[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7112290649774386037[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7112290649774386037[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7112290649774386037[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7112290649774386037[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7112290649774386037[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7112290649774386037[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7112290649774386037[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7112290649774386037[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7112290649774386037[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7112290649774386037[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7112290649774386037[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7112290649774386037[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7112290649774386037[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7112290649774386037[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7112290649774386037[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7112290649774386037[56] = 0;
   out_7112290649774386037[57] = 0;
   out_7112290649774386037[58] = 0;
   out_7112290649774386037[59] = 0;
   out_7112290649774386037[60] = 0;
   out_7112290649774386037[61] = 0;
   out_7112290649774386037[62] = 0;
   out_7112290649774386037[63] = 1;
}
void h_25(double *state, double *unused, double *out_6007319110719284431) {
   out_6007319110719284431[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3515057580342967796) {
   out_3515057580342967796[0] = 0;
   out_3515057580342967796[1] = 0;
   out_3515057580342967796[2] = 0;
   out_3515057580342967796[3] = 0;
   out_3515057580342967796[4] = 0;
   out_3515057580342967796[5] = 0;
   out_3515057580342967796[6] = 1;
   out_3515057580342967796[7] = 0;
}
void h_24(double *state, double *unused, double *out_3469855358222195064) {
   out_3469855358222195064[0] = state[4];
   out_3469855358222195064[1] = state[5];
}
void H_24(double *state, double *unused, double *out_843611343158634957) {
   out_843611343158634957[0] = 0;
   out_843611343158634957[1] = 0;
   out_843611343158634957[2] = 0;
   out_843611343158634957[3] = 0;
   out_843611343158634957[4] = 1;
   out_843611343158634957[5] = 0;
   out_843611343158634957[6] = 0;
   out_843611343158634957[7] = 0;
   out_843611343158634957[8] = 0;
   out_843611343158634957[9] = 0;
   out_843611343158634957[10] = 0;
   out_843611343158634957[11] = 0;
   out_843611343158634957[12] = 0;
   out_843611343158634957[13] = 1;
   out_843611343158634957[14] = 0;
   out_843611343158634957[15] = 0;
}
void h_30(double *state, double *unused, double *out_6913367526439244072) {
   out_6913367526439244072[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5317787582417400540) {
   out_5317787582417400540[0] = 0;
   out_5317787582417400540[1] = 0;
   out_5317787582417400540[2] = 0;
   out_5317787582417400540[3] = 0;
   out_5317787582417400540[4] = 1;
   out_5317787582417400540[5] = 0;
   out_5317787582417400540[6] = 0;
   out_5317787582417400540[7] = 0;
}
void h_26(double *state, double *unused, double *out_5065273851963438732) {
   out_5065273851963438732[0] = state[7];
}
void H_26(double *state, double *unused, double *out_9191892976686568140) {
   out_9191892976686568140[0] = 0;
   out_9191892976686568140[1] = 0;
   out_9191892976686568140[2] = 0;
   out_9191892976686568140[3] = 0;
   out_9191892976686568140[4] = 0;
   out_9191892976686568140[5] = 0;
   out_9191892976686568140[6] = 0;
   out_9191892976686568140[7] = 1;
}
void h_27(double *state, double *unused, double *out_5183281245896815476) {
   out_5183281245896815476[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3015823694054081597) {
   out_3015823694054081597[0] = 0;
   out_3015823694054081597[1] = 0;
   out_3015823694054081597[2] = 0;
   out_3015823694054081597[3] = 1;
   out_3015823694054081597[4] = 0;
   out_3015823694054081597[5] = 0;
   out_3015823694054081597[6] = 0;
   out_3015823694054081597[7] = 0;
}
void h_29(double *state, double *unused, double *out_3456596230821726408) {
   out_3456596230821726408[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1493988813288568587) {
   out_1493988813288568587[0] = 0;
   out_1493988813288568587[1] = 1;
   out_1493988813288568587[2] = 0;
   out_1493988813288568587[3] = 0;
   out_1493988813288568587[4] = 0;
   out_1493988813288568587[5] = 0;
   out_1493988813288568587[6] = 0;
   out_1493988813288568587[7] = 0;
}
void h_28(double *state, double *unused, double *out_2631209287820020534) {
   out_2631209287820020534[0] = state[5];
   out_2631209287820020534[1] = state[6];
}
void H_28(double *state, double *unused, double *out_7861443821395129823) {
   out_7861443821395129823[0] = 0;
   out_7861443821395129823[1] = 0;
   out_7861443821395129823[2] = 0;
   out_7861443821395129823[3] = 0;
   out_7861443821395129823[4] = 0;
   out_7861443821395129823[5] = 1;
   out_7861443821395129823[6] = 0;
   out_7861443821395129823[7] = 0;
   out_7861443821395129823[8] = 0;
   out_7861443821395129823[9] = 0;
   out_7861443821395129823[10] = 0;
   out_7861443821395129823[11] = 0;
   out_7861443821395129823[12] = 0;
   out_7861443821395129823[13] = 0;
   out_7861443821395129823[14] = 1;
   out_7861443821395129823[15] = 0;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6769788503972096709) {
  err_fun(nom_x, delta_x, out_6769788503972096709);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_152269297933425103) {
  inv_err_fun(nom_x, true_x, out_152269297933425103);
}
void car_H_mod_fun(double *state, double *out_4881300373756157474) {
  H_mod_fun(state, out_4881300373756157474);
}
void car_f_fun(double *state, double dt, double *out_6919783745829379099) {
  f_fun(state,  dt, out_6919783745829379099);
}
void car_F_fun(double *state, double dt, double *out_7112290649774386037) {
  F_fun(state,  dt, out_7112290649774386037);
}
void car_h_25(double *state, double *unused, double *out_6007319110719284431) {
  h_25(state, unused, out_6007319110719284431);
}
void car_H_25(double *state, double *unused, double *out_3515057580342967796) {
  H_25(state, unused, out_3515057580342967796);
}
void car_h_24(double *state, double *unused, double *out_3469855358222195064) {
  h_24(state, unused, out_3469855358222195064);
}
void car_H_24(double *state, double *unused, double *out_843611343158634957) {
  H_24(state, unused, out_843611343158634957);
}
void car_h_30(double *state, double *unused, double *out_6913367526439244072) {
  h_30(state, unused, out_6913367526439244072);
}
void car_H_30(double *state, double *unused, double *out_5317787582417400540) {
  H_30(state, unused, out_5317787582417400540);
}
void car_h_26(double *state, double *unused, double *out_5065273851963438732) {
  h_26(state, unused, out_5065273851963438732);
}
void car_H_26(double *state, double *unused, double *out_9191892976686568140) {
  H_26(state, unused, out_9191892976686568140);
}
void car_h_27(double *state, double *unused, double *out_5183281245896815476) {
  h_27(state, unused, out_5183281245896815476);
}
void car_H_27(double *state, double *unused, double *out_3015823694054081597) {
  H_27(state, unused, out_3015823694054081597);
}
void car_h_29(double *state, double *unused, double *out_3456596230821726408) {
  h_29(state, unused, out_3456596230821726408);
}
void car_H_29(double *state, double *unused, double *out_1493988813288568587) {
  H_29(state, unused, out_1493988813288568587);
}
void car_h_28(double *state, double *unused, double *out_2631209287820020534) {
  h_28(state, unused, out_2631209287820020534);
}
void car_H_28(double *state, double *unused, double *out_7861443821395129823) {
  H_28(state, unused, out_7861443821395129823);
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
