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
void live_H(double *in_vec, double *out_7176771882123068031);
void live_err_fun(double *nom_x, double *delta_x, double *out_7383003451745682886);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8043250382326694273);
void live_H_mod_fun(double *state, double *out_3022064996999117402);
void live_f_fun(double *state, double dt, double *out_211128429486972418);
void live_F_fun(double *state, double dt, double *out_7477901336729587113);
void live_h_4(double *state, double *unused, double *out_1416724246099778923);
void live_H_4(double *state, double *unused, double *out_640460847784457531);
void live_h_9(double *state, double *unused, double *out_5267497251750324843);
void live_H_9(double *state, double *unused, double *out_399271201154866886);
void live_h_10(double *state, double *unused, double *out_2133571045890165454);
void live_H_10(double *state, double *unused, double *out_6608212112768862026);
void live_h_12(double *state, double *unused, double *out_2579549306557761547);
void live_H_12(double *state, double *unused, double *out_19361822736863864);
void live_h_35(double *state, double *unused, double *out_5125780411757793974);
void live_H_35(double *state, double *unused, double *out_2726201209588149845);
void live_h_32(double *state, double *unused, double *out_7940142825117524696);
void live_H_32(double *state, double *unused, double *out_1614803566144957043);
void live_h_13(double *state, double *unused, double *out_453097342475321776);
void live_H_13(double *state, double *unused, double *out_1585131618163734730);
void live_h_14(double *state, double *unused, double *out_5267497251750324843);
void live_H_14(double *state, double *unused, double *out_399271201154866886);
void live_h_33(double *state, double *unused, double *out_2148794914329694256);
void live_H_33(double *state, double *unused, double *out_5876758214227007449);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}