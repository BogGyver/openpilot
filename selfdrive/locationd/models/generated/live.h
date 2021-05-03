#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_3(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_19(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6440091693162022888);
void live_err_fun(double *nom_x, double *delta_x, double *out_4045068409998732168);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1130725816871523314);
void live_H_mod_fun(double *state, double *out_9057055102025961707);
void live_f_fun(double *state, double dt, double *out_2924345871061403756);
void live_F_fun(double *state, double dt, double *out_1613889119122228654);
void live_h_3(double *state, double *unused, double *out_5439543506784062287);
void live_H_3(double *state, double *unused, double *out_3280548030947440156);
void live_h_4(double *state, double *unused, double *out_1525623624983305193);
void live_H_4(double *state, double *unused, double *out_7560605787500324739);
void live_h_9(double *state, double *unused, double *out_149483631174059161);
void live_H_9(double *state, double *unused, double *out_2880753929280854046);
void live_h_10(double *state, double *unused, double *out_5544878077703004476);
void live_H_10(double *state, double *unused, double *out_1592384890266115533);
void live_h_12(double *state, double *unused, double *out_1573182497905511634);
void live_H_12(double *state, double *unused, double *out_8779582774470017308);
void live_h_31(double *state, double *unused, double *out_1879484515421114513);
void live_H_31(double *state, double *unused, double *out_7056315166629069001);
void live_h_32(double *state, double *unused, double *out_8825490026557837445);
void live_H_32(double *state, double *unused, double *out_9006816888276820648);
void live_h_13(double *state, double *unused, double *out_5489966362556065194);
void live_H_13(double *state, double *unused, double *out_3763103836578933447);
void live_h_14(double *state, double *unused, double *out_149483631174059161);
void live_H_14(double *state, double *unused, double *out_2880753929280854046);
void live_h_19(double *state, double *unused, double *out_5895963450060674577);
void live_H_19(double *state, double *unused, double *out_7313433780383285227);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}