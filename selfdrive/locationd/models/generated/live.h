#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_4028264744687795352);
void live_err_fun(double *nom_x, double *delta_x, double *out_8435331516567488924);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5002002164599025613);
void live_H_mod_fun(double *state, double *out_3440322527295895629);
void live_f_fun(double *state, double dt, double *out_6600220164658986461);
void live_F_fun(double *state, double dt, double *out_6389863516536335469);
void live_h_4(double *state, double *unused, double *out_7661006871823619503);
void live_H_4(double *state, double *unused, double *out_7501541336268905998);
void live_h_9(double *state, double *unused, double *out_4926267146494518877);
void live_H_9(double *state, double *unused, double *out_7260351689639315353);
void live_h_10(double *state, double *unused, double *out_652311280382873356);
void live_H_10(double *state, double *unused, double *out_6526497181785554798);
void live_h_12(double *state, double *unused, double *out_4327059784582922294);
void live_H_12(double *state, double *unused, double *out_6880442311221312331);
void live_h_31(double *state, double *unused, double *out_5720054282446494434);
void live_H_31(double *state, double *unused, double *out_4134879278896298622);
void live_h_32(double *state, double *unused, double *out_6542287694810245299);
void live_H_32(double *state, double *unused, double *out_587034938909139893);
void live_h_13(double *state, double *unused, double *out_2498144898772092043);
void live_H_13(double *state, double *unused, double *out_482060401086535411);
void live_h_14(double *state, double *unused, double *out_4926267146494518877);
void live_H_14(double *state, double *unused, double *out_7260351689639315353);
void live_h_33(double *state, double *unused, double *out_4451523588948760961);
void live_H_33(double *state, double *unused, double *out_984322274257441018);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}