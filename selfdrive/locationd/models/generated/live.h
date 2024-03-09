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
void live_H(double *in_vec, double *out_6971914640542061287);
void live_err_fun(double *nom_x, double *delta_x, double *out_6189662298219493583);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7314469681338286083);
void live_H_mod_fun(double *state, double *out_2317264427215604781);
void live_f_fun(double *state, double dt, double *out_6421330834293840922);
void live_F_fun(double *state, double dt, double *out_8335401751809609889);
void live_h_4(double *state, double *unused, double *out_5080936158019373930);
void live_H_4(double *state, double *unused, double *out_2638295698616869394);
void live_h_9(double *state, double *unused, double *out_4893286284277234855);
void live_H_9(double *state, double *unused, double *out_4648923236647578076);
void live_h_10(double *state, double *unused, double *out_1138825551532232374);
void live_H_10(double *state, double *unused, double *out_4414796935612752637);
void live_h_12(double *state, double *unused, double *out_6535681285043612047);
void live_H_12(double *state, double *unused, double *out_2381160709415092401);
void live_h_35(double *state, double *unused, double *out_5008249702129854706);
void live_H_35(double *state, double *unused, double *out_5126723741740106110);
void live_h_32(double *state, double *unused, double *out_8400234986997979728);
void live_H_32(double *state, double *unused, double *out_3490332878916171088);
void live_h_13(double *state, double *unused, double *out_719563951820547242);
void live_H_13(double *state, double *unused, double *out_7339299324380762044);
void live_h_14(double *state, double *unused, double *out_4893286284277234855);
void live_H_14(double *state, double *unused, double *out_4648923236647578076);
void live_h_33(double *state, double *unused, double *out_3264372632123712536);
void live_H_33(double *state, double *unused, double *out_8277280746378963714);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}