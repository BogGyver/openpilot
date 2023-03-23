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
void live_H(double *in_vec, double *out_75171994648033650);
void live_err_fun(double *nom_x, double *delta_x, double *out_7890645936796359548);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5987157107229293753);
void live_H_mod_fun(double *state, double *out_814311292743292767);
void live_f_fun(double *state, double dt, double *out_9004231435331892546);
void live_F_fun(double *state, double dt, double *out_3729569181691432612);
void live_h_4(double *state, double *unused, double *out_2659404755543687570);
void live_H_4(double *state, double *unused, double *out_3231739655702570018);
void live_h_9(double *state, double *unused, double *out_5898723650944211759);
void live_H_9(double *state, double *unused, double *out_7388907392057347501);
void live_h_10(double *state, double *unused, double *out_6935302867683902446);
void live_H_10(double *state, double *unused, double *out_12402863138039837);
void live_h_12(double *state, double *unused, double *out_3215226667935742235);
void live_H_12(double *state, double *unused, double *out_2610640630654976351);
void live_h_31(double *state, double *unused, double *out_3488530232831541611);
void live_H_31(double *state, double *unused, double *out_134922401670037358);
void live_h_32(double *state, double *unused, double *out_7606020131008712511);
void live_H_32(double *state, double *unused, double *out_6779942102024078998);
void live_h_13(double *state, double *unused, double *out_1987864673995626961);
void live_H_13(double *state, double *unused, double *out_608359196675629431);
void live_h_14(double *state, double *unused, double *out_5898723650944211759);
void live_H_14(double *state, double *unused, double *out_7388907392057347501);
void live_h_33(double *state, double *unused, double *out_3040627374087545041);
void live_H_33(double *state, double *unused, double *out_3285479406308894962);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}