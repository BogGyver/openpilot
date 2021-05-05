#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6769788503972096709);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_152269297933425103);
void car_H_mod_fun(double *state, double *out_4881300373756157474);
void car_f_fun(double *state, double dt, double *out_6919783745829379099);
void car_F_fun(double *state, double dt, double *out_7112290649774386037);
void car_h_25(double *state, double *unused, double *out_6007319110719284431);
void car_H_25(double *state, double *unused, double *out_3515057580342967796);
void car_h_24(double *state, double *unused, double *out_3469855358222195064);
void car_H_24(double *state, double *unused, double *out_843611343158634957);
void car_h_30(double *state, double *unused, double *out_6913367526439244072);
void car_H_30(double *state, double *unused, double *out_5317787582417400540);
void car_h_26(double *state, double *unused, double *out_5065273851963438732);
void car_H_26(double *state, double *unused, double *out_9191892976686568140);
void car_h_27(double *state, double *unused, double *out_5183281245896815476);
void car_H_27(double *state, double *unused, double *out_3015823694054081597);
void car_h_29(double *state, double *unused, double *out_3456596230821726408);
void car_H_29(double *state, double *unused, double *out_1493988813288568587);
void car_h_28(double *state, double *unused, double *out_2631209287820020534);
void car_H_28(double *state, double *unused, double *out_7861443821395129823);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}