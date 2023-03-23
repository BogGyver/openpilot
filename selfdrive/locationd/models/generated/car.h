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
void car_err_fun(double *nom_x, double *delta_x, double *out_7853339013055020703);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4568207359509177387);
void car_H_mod_fun(double *state, double *out_400617013284271733);
void car_f_fun(double *state, double dt, double *out_536377625142687133);
void car_F_fun(double *state, double dt, double *out_8637102387366365346);
void car_h_25(double *state, double *unused, double *out_3784825874740371840);
void car_H_25(double *state, double *unused, double *out_3254223637679582827);
void car_h_24(double *state, double *unused, double *out_5977641087208495030);
void car_H_24(double *state, double *unused, double *out_3724681119722921551);
void car_h_30(double *state, double *unused, double *out_7891082972618828840);
void car_H_30(double *state, double *unused, double *out_3383562584822822897);
void car_h_26(double *state, double *unused, double *out_2889409704957983195);
void car_H_26(double *state, double *unused, double *out_6995726956553639051);
void car_h_27(double *state, double *unused, double *out_1216847392920473805);
void car_H_27(double *state, double *unused, double *out_5558325896623247808);
void car_h_29(double *state, double *unused, double *out_5553987833429877131);
void car_H_29(double *state, double *unused, double *out_2873331240508430713);
void car_h_28(double *state, double *unused, double *out_273337540018811370);
void car_H_28(double *state, double *unused, double *out_909700968943104462);
void car_h_31(double *state, double *unused, double *out_4108113620144151519);
void car_H_31(double *state, double *unused, double *out_3223577675802622399);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}