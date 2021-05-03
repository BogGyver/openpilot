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
void car_err_fun(double *nom_x, double *delta_x, double *out_283129011095924522);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4497310152589916621);
void car_H_mod_fun(double *state, double *out_1841431657313326858);
void car_f_fun(double *state, double dt, double *out_7063787066656637565);
void car_F_fun(double *state, double dt, double *out_7660651947569880398);
void car_h_25(double *state, double *unused, double *out_5420055961677953059);
void car_H_25(double *state, double *unused, double *out_864586791604080428);
void car_h_24(double *state, double *unused, double *out_2065971680471068193);
void car_H_24(double *state, double *unused, double *out_5332772925216162998);
void car_h_30(double *state, double *unused, double *out_7500630675480575444);
void car_H_30(double *state, double *unused, double *out_7968258371156287908);
void car_h_26(double *state, double *unused, double *out_1254179618020401976);
void car_H_26(double *state, double *unused, double *out_4840006363335128845);
void car_h_27(double *state, double *unused, double *out_5865726006614306196);
void car_H_27(double *state, double *unused, double *out_6680676383319662596);
void car_h_29(double *state, double *unused, double *out_4896225354671877793);
void car_H_29(double *state, double *unused, double *out_4144459602027455955);
void car_h_28(double *state, double *unused, double *out_8461947836500736621);
void car_H_28(double *state, double *unused, double *out_1685059553020331868);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}