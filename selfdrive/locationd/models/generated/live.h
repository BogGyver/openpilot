#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1322097756330465925);
void live_err_fun(double *nom_x, double *delta_x, double *out_6016948068371293090);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4185247525971947783);
void live_H_mod_fun(double *state, double *out_1956496249937155735);
void live_f_fun(double *state, double dt, double *out_219416794619969174);
void live_F_fun(double *state, double dt, double *out_5435754757303912807);
void live_h_4(double *state, double *unused, double *out_4343585568778497722);
void live_H_4(double *state, double *unused, double *out_3893613413986227078);
void live_h_9(double *state, double *unused, double *out_8528586901421267304);
void live_H_9(double *state, double *unused, double *out_7265911724458877068);
void live_h_10(double *state, double *unused, double *out_7028108485377770697);
void live_H_10(double *state, double *unused, double *out_248256028817931248);
void live_h_12(double *state, double *unused, double *out_7355422993027711480);
void live_H_12(double *state, double *unused, double *out_2487644963056505918);
void live_h_35(double *state, double *unused, double *out_7685232196774128584);
void live_H_35(double *state, double *unused, double *out_7260275471358834454);
void live_h_32(double *state, double *unused, double *out_6841005505128934852);
void live_H_32(double *state, double *unused, double *out_7391440256299574923);
void live_h_13(double *state, double *unused, double *out_6591997094849217203);
void live_H_13(double *state, double *unused, double *out_512175357445750065);
void live_h_14(double *state, double *unused, double *out_8528586901421267304);
void live_H_14(double *state, double *unused, double *out_7265911724458877068);
void live_h_33(double *state, double *unused, double *out_4758804836398744126);
void live_H_33(double *state, double *unused, double *out_8035911597711859558);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}