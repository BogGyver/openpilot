#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_198445823836928788);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7598262341626218705);
void car_H_mod_fun(double *state, double *out_2149329292292161718);
void car_f_fun(double *state, double dt, double *out_7137556374309530724);
void car_F_fun(double *state, double dt, double *out_8393538857825963338);
void car_h_25(double *state, double *unused, double *out_1913767349031954091);
void car_H_25(double *state, double *unused, double *out_3307554411264577435);
void car_h_24(double *state, double *unused, double *out_1050780426769953054);
void car_H_24(double *state, double *unused, double *out_8054063738231086469);
void car_h_30(double *state, double *unused, double *out_8769711467856925084);
void car_H_30(double *state, double *unused, double *out_789221452757328808);
void car_h_26(double *state, double *unused, double *out_4568584500733172853);
void car_H_26(double *state, double *unused, double *out_7049057730138633659);
void car_h_27(double *state, double *unused, double *out_2359437393968307228);
void car_H_27(double *state, double *unused, double *out_1434372618426614409);
void car_h_29(double *state, double *unused, double *out_4451125390017301585);
void car_H_29(double *state, double *unused, double *out_278990108442936624);
void car_h_28(double *state, double *unused, double *out_6641001235211967736);
void car_H_28(double *state, double *unused, double *out_5361389125512467198);
void car_h_31(double *state, double *unused, double *out_8651704073923548340);
void car_H_31(double *state, double *unused, double *out_3276908449387617007);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}