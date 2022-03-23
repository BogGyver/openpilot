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
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1475588756916852268);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3596983127668324109);
void car_H_mod_fun(double *state, double *out_3389099407762428077);
void car_f_fun(double *state, double dt, double *out_5045286729280014390);
void car_F_fun(double *state, double dt, double *out_8901810177871386405);
void car_h_25(double *state, double *unused, double *out_2445776787221609462);
void car_H_25(double *state, double *unused, double *out_5198710936402375667);
void car_h_24(double *state, double *unused, double *out_4307300272860009815);
void car_H_24(double *state, double *unused, double *out_8379218272279469097);
void car_h_30(double *state, double *unused, double *out_5317488469599062979);
void car_H_30(double *state, double *unused, double *out_8720336807179567751);
void car_h_26(double *state, double *unused, double *out_7250970116551773549);
void car_H_26(double *state, double *unused, double *out_8940214255276431891);
void car_h_27(double *state, double *unused, double *out_970496502427460554);
void car_H_27(double *state, double *unused, double *out_6545573495379142840);
void car_h_29(double *state, double *unused, double *out_315815201938217334);
void car_H_29(double *state, double *unused, double *out_9216175922215591681);
void car_h_28(double *state, double *unused, double *out_4031176765233385091);
void car_H_28(double *state, double *unused, double *out_7252545650650265430);
void car_h_31(double *state, double *unused, double *out_5435495863532439723);
void car_H_31(double *state, double *unused, double *out_8880321716199768249);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}