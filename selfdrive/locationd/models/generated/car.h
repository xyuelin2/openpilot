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
void car_err_fun(double *nom_x, double *delta_x, double *out_6969881541203456864);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1550632269520691561);
void car_H_mod_fun(double *state, double *out_4664115282963326508);
void car_f_fun(double *state, double dt, double *out_3600870033497684324);
void car_F_fun(double *state, double dt, double *out_2816270813315995545);
void car_h_25(double *state, double *unused, double *out_751319912457213132);
void car_H_25(double *state, double *unused, double *out_1682570372359171018);
void car_h_24(double *state, double *unused, double *out_8433912697962430457);
void car_H_24(double *state, double *unused, double *out_8916819690476358536);
void car_h_30(double *state, double *unused, double *out_4849623210736584996);
void car_H_30(double *state, double *unused, double *out_4200903330866419645);
void car_h_26(double *state, double *unused, double *out_2228954837399959704);
void car_H_26(double *state, double *unused, double *out_2058932946514885206);
void car_h_27(double *state, double *unused, double *out_9156976732101439785);
void car_H_27(double *state, double *unused, double *out_2026140019065994734);
void car_h_29(double *state, double *unused, double *out_152050056924260649);
void car_H_29(double *state, double *unused, double *out_4711134675180811829);
void car_h_28(double *state, double *unused, double *out_5008203500364451323);
void car_H_28(double *state, double *unused, double *out_371264341888718745);
void car_h_31(double *state, double *unused, double *out_650644545693243836);
void car_H_31(double *state, double *unused, double *out_2685141048748236682);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}