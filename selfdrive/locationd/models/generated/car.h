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
void car_err_fun(double *nom_x, double *delta_x, double *out_4112143657443420345);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1448447221229649002);
void car_H_mod_fun(double *state, double *out_6589730968564999447);
void car_f_fun(double *state, double dt, double *out_6843342043308437648);
void car_F_fun(double *state, double dt, double *out_5517739003727465901);
void car_h_25(double *state, double *unused, double *out_3400981906576064285);
void car_H_25(double *state, double *unused, double *out_2200893013917596422);
void car_h_24(double *state, double *unused, double *out_3243230712214609592);
void car_H_24(double *state, double *unused, double *out_1781430137480171476);
void car_h_30(double *state, double *unused, double *out_236467621677958065);
void car_H_30(double *state, double *unused, double *out_9117583355409213177);
void car_h_26(double *state, double *unused, double *out_366848262998519452);
void car_H_26(double *state, double *unused, double *out_1540610304956459802);
void car_h_27(double *state, double *unused, double *out_2643131761415245475);
void car_H_27(double *state, double *unused, double *out_6942820043608788266);
void car_h_29(double *state, double *unused, double *out_5238140889338803710);
void car_H_29(double *state, double *unused, double *out_5229457316739237233);
void car_h_28(double *state, double *unused, double *out_7381277810922059405);
void car_H_28(double *state, double *unused, double *out_147058299669706659);
void car_h_31(double *state, double *unused, double *out_6194173636447207953);
void car_H_31(double *state, double *unused, double *out_2231538975794556850);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}