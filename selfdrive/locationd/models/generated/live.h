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
void live_H(double *in_vec, double *out_4710942819665481182);
void live_err_fun(double *nom_x, double *delta_x, double *out_4861823975196145972);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8125544546544067036);
void live_H_mod_fun(double *state, double *out_6904388490792526769);
void live_f_fun(double *state, double dt, double *out_3365756317685635482);
void live_F_fun(double *state, double dt, double *out_6622079963924894762);
void live_h_3(double *state, double *unused, double *out_4273251906995718702);
void live_H_3(double *state, double *unused, double *out_3263360507854630798);
void live_h_4(double *state, double *unused, double *out_6006996445108386487);
void live_H_4(double *state, double *unused, double *out_5370414345011815521);
void live_h_9(double *state, double *unused, double *out_2353671559511824253);
void live_H_9(double *state, double *unused, double *out_3707794896192023300);
void live_h_10(double *state, double *unused, double *out_23610964328663066);
void live_H_10(double *state, double *unused, double *out_2353610387599420621);
void live_h_12(double *state, double *unused, double *out_8704239366125542035);
void live_H_12(double *state, double *unused, double *out_7476969856751025090);
void live_h_31(double *state, double *unused, double *out_320527856156286463);
void live_H_31(double *state, double *unused, double *out_4801880081607605269);
void live_h_32(double *state, double *unused, double *out_6370737567058137512);
void live_H_32(double *state, double *unused, double *out_1498485662652795917);
void live_h_13(double *state, double *unused, double *out_7949254008253023402);
void live_H_13(double *state, double *unused, double *out_3914911562115107662);
void live_h_14(double *state, double *unused, double *out_2353671559511824253);
void live_H_14(double *state, double *unused, double *out_3707794896192023300);
void live_h_19(double *state, double *unused, double *out_8638976190319524959);
void live_H_19(double *state, double *unused, double *out_724884954910407881);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}