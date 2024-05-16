#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6969881541203456864) {
   out_6969881541203456864[0] = delta_x[0] + nom_x[0];
   out_6969881541203456864[1] = delta_x[1] + nom_x[1];
   out_6969881541203456864[2] = delta_x[2] + nom_x[2];
   out_6969881541203456864[3] = delta_x[3] + nom_x[3];
   out_6969881541203456864[4] = delta_x[4] + nom_x[4];
   out_6969881541203456864[5] = delta_x[5] + nom_x[5];
   out_6969881541203456864[6] = delta_x[6] + nom_x[6];
   out_6969881541203456864[7] = delta_x[7] + nom_x[7];
   out_6969881541203456864[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1550632269520691561) {
   out_1550632269520691561[0] = -nom_x[0] + true_x[0];
   out_1550632269520691561[1] = -nom_x[1] + true_x[1];
   out_1550632269520691561[2] = -nom_x[2] + true_x[2];
   out_1550632269520691561[3] = -nom_x[3] + true_x[3];
   out_1550632269520691561[4] = -nom_x[4] + true_x[4];
   out_1550632269520691561[5] = -nom_x[5] + true_x[5];
   out_1550632269520691561[6] = -nom_x[6] + true_x[6];
   out_1550632269520691561[7] = -nom_x[7] + true_x[7];
   out_1550632269520691561[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4664115282963326508) {
   out_4664115282963326508[0] = 1.0;
   out_4664115282963326508[1] = 0;
   out_4664115282963326508[2] = 0;
   out_4664115282963326508[3] = 0;
   out_4664115282963326508[4] = 0;
   out_4664115282963326508[5] = 0;
   out_4664115282963326508[6] = 0;
   out_4664115282963326508[7] = 0;
   out_4664115282963326508[8] = 0;
   out_4664115282963326508[9] = 0;
   out_4664115282963326508[10] = 1.0;
   out_4664115282963326508[11] = 0;
   out_4664115282963326508[12] = 0;
   out_4664115282963326508[13] = 0;
   out_4664115282963326508[14] = 0;
   out_4664115282963326508[15] = 0;
   out_4664115282963326508[16] = 0;
   out_4664115282963326508[17] = 0;
   out_4664115282963326508[18] = 0;
   out_4664115282963326508[19] = 0;
   out_4664115282963326508[20] = 1.0;
   out_4664115282963326508[21] = 0;
   out_4664115282963326508[22] = 0;
   out_4664115282963326508[23] = 0;
   out_4664115282963326508[24] = 0;
   out_4664115282963326508[25] = 0;
   out_4664115282963326508[26] = 0;
   out_4664115282963326508[27] = 0;
   out_4664115282963326508[28] = 0;
   out_4664115282963326508[29] = 0;
   out_4664115282963326508[30] = 1.0;
   out_4664115282963326508[31] = 0;
   out_4664115282963326508[32] = 0;
   out_4664115282963326508[33] = 0;
   out_4664115282963326508[34] = 0;
   out_4664115282963326508[35] = 0;
   out_4664115282963326508[36] = 0;
   out_4664115282963326508[37] = 0;
   out_4664115282963326508[38] = 0;
   out_4664115282963326508[39] = 0;
   out_4664115282963326508[40] = 1.0;
   out_4664115282963326508[41] = 0;
   out_4664115282963326508[42] = 0;
   out_4664115282963326508[43] = 0;
   out_4664115282963326508[44] = 0;
   out_4664115282963326508[45] = 0;
   out_4664115282963326508[46] = 0;
   out_4664115282963326508[47] = 0;
   out_4664115282963326508[48] = 0;
   out_4664115282963326508[49] = 0;
   out_4664115282963326508[50] = 1.0;
   out_4664115282963326508[51] = 0;
   out_4664115282963326508[52] = 0;
   out_4664115282963326508[53] = 0;
   out_4664115282963326508[54] = 0;
   out_4664115282963326508[55] = 0;
   out_4664115282963326508[56] = 0;
   out_4664115282963326508[57] = 0;
   out_4664115282963326508[58] = 0;
   out_4664115282963326508[59] = 0;
   out_4664115282963326508[60] = 1.0;
   out_4664115282963326508[61] = 0;
   out_4664115282963326508[62] = 0;
   out_4664115282963326508[63] = 0;
   out_4664115282963326508[64] = 0;
   out_4664115282963326508[65] = 0;
   out_4664115282963326508[66] = 0;
   out_4664115282963326508[67] = 0;
   out_4664115282963326508[68] = 0;
   out_4664115282963326508[69] = 0;
   out_4664115282963326508[70] = 1.0;
   out_4664115282963326508[71] = 0;
   out_4664115282963326508[72] = 0;
   out_4664115282963326508[73] = 0;
   out_4664115282963326508[74] = 0;
   out_4664115282963326508[75] = 0;
   out_4664115282963326508[76] = 0;
   out_4664115282963326508[77] = 0;
   out_4664115282963326508[78] = 0;
   out_4664115282963326508[79] = 0;
   out_4664115282963326508[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3600870033497684324) {
   out_3600870033497684324[0] = state[0];
   out_3600870033497684324[1] = state[1];
   out_3600870033497684324[2] = state[2];
   out_3600870033497684324[3] = state[3];
   out_3600870033497684324[4] = state[4];
   out_3600870033497684324[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3600870033497684324[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3600870033497684324[7] = state[7];
   out_3600870033497684324[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2816270813315995545) {
   out_2816270813315995545[0] = 1;
   out_2816270813315995545[1] = 0;
   out_2816270813315995545[2] = 0;
   out_2816270813315995545[3] = 0;
   out_2816270813315995545[4] = 0;
   out_2816270813315995545[5] = 0;
   out_2816270813315995545[6] = 0;
   out_2816270813315995545[7] = 0;
   out_2816270813315995545[8] = 0;
   out_2816270813315995545[9] = 0;
   out_2816270813315995545[10] = 1;
   out_2816270813315995545[11] = 0;
   out_2816270813315995545[12] = 0;
   out_2816270813315995545[13] = 0;
   out_2816270813315995545[14] = 0;
   out_2816270813315995545[15] = 0;
   out_2816270813315995545[16] = 0;
   out_2816270813315995545[17] = 0;
   out_2816270813315995545[18] = 0;
   out_2816270813315995545[19] = 0;
   out_2816270813315995545[20] = 1;
   out_2816270813315995545[21] = 0;
   out_2816270813315995545[22] = 0;
   out_2816270813315995545[23] = 0;
   out_2816270813315995545[24] = 0;
   out_2816270813315995545[25] = 0;
   out_2816270813315995545[26] = 0;
   out_2816270813315995545[27] = 0;
   out_2816270813315995545[28] = 0;
   out_2816270813315995545[29] = 0;
   out_2816270813315995545[30] = 1;
   out_2816270813315995545[31] = 0;
   out_2816270813315995545[32] = 0;
   out_2816270813315995545[33] = 0;
   out_2816270813315995545[34] = 0;
   out_2816270813315995545[35] = 0;
   out_2816270813315995545[36] = 0;
   out_2816270813315995545[37] = 0;
   out_2816270813315995545[38] = 0;
   out_2816270813315995545[39] = 0;
   out_2816270813315995545[40] = 1;
   out_2816270813315995545[41] = 0;
   out_2816270813315995545[42] = 0;
   out_2816270813315995545[43] = 0;
   out_2816270813315995545[44] = 0;
   out_2816270813315995545[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2816270813315995545[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2816270813315995545[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2816270813315995545[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2816270813315995545[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2816270813315995545[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2816270813315995545[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2816270813315995545[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2816270813315995545[53] = -9.8000000000000007*dt;
   out_2816270813315995545[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2816270813315995545[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2816270813315995545[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2816270813315995545[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2816270813315995545[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2816270813315995545[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2816270813315995545[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2816270813315995545[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2816270813315995545[62] = 0;
   out_2816270813315995545[63] = 0;
   out_2816270813315995545[64] = 0;
   out_2816270813315995545[65] = 0;
   out_2816270813315995545[66] = 0;
   out_2816270813315995545[67] = 0;
   out_2816270813315995545[68] = 0;
   out_2816270813315995545[69] = 0;
   out_2816270813315995545[70] = 1;
   out_2816270813315995545[71] = 0;
   out_2816270813315995545[72] = 0;
   out_2816270813315995545[73] = 0;
   out_2816270813315995545[74] = 0;
   out_2816270813315995545[75] = 0;
   out_2816270813315995545[76] = 0;
   out_2816270813315995545[77] = 0;
   out_2816270813315995545[78] = 0;
   out_2816270813315995545[79] = 0;
   out_2816270813315995545[80] = 1;
}
void h_25(double *state, double *unused, double *out_751319912457213132) {
   out_751319912457213132[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1682570372359171018) {
   out_1682570372359171018[0] = 0;
   out_1682570372359171018[1] = 0;
   out_1682570372359171018[2] = 0;
   out_1682570372359171018[3] = 0;
   out_1682570372359171018[4] = 0;
   out_1682570372359171018[5] = 0;
   out_1682570372359171018[6] = 1;
   out_1682570372359171018[7] = 0;
   out_1682570372359171018[8] = 0;
}
void h_24(double *state, double *unused, double *out_8433912697962430457) {
   out_8433912697962430457[0] = state[4];
   out_8433912697962430457[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8916819690476358536) {
   out_8916819690476358536[0] = 0;
   out_8916819690476358536[1] = 0;
   out_8916819690476358536[2] = 0;
   out_8916819690476358536[3] = 0;
   out_8916819690476358536[4] = 1;
   out_8916819690476358536[5] = 0;
   out_8916819690476358536[6] = 0;
   out_8916819690476358536[7] = 0;
   out_8916819690476358536[8] = 0;
   out_8916819690476358536[9] = 0;
   out_8916819690476358536[10] = 0;
   out_8916819690476358536[11] = 0;
   out_8916819690476358536[12] = 0;
   out_8916819690476358536[13] = 0;
   out_8916819690476358536[14] = 1;
   out_8916819690476358536[15] = 0;
   out_8916819690476358536[16] = 0;
   out_8916819690476358536[17] = 0;
}
void h_30(double *state, double *unused, double *out_4849623210736584996) {
   out_4849623210736584996[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4200903330866419645) {
   out_4200903330866419645[0] = 0;
   out_4200903330866419645[1] = 0;
   out_4200903330866419645[2] = 0;
   out_4200903330866419645[3] = 0;
   out_4200903330866419645[4] = 1;
   out_4200903330866419645[5] = 0;
   out_4200903330866419645[6] = 0;
   out_4200903330866419645[7] = 0;
   out_4200903330866419645[8] = 0;
}
void h_26(double *state, double *unused, double *out_2228954837399959704) {
   out_2228954837399959704[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2058932946514885206) {
   out_2058932946514885206[0] = 0;
   out_2058932946514885206[1] = 0;
   out_2058932946514885206[2] = 0;
   out_2058932946514885206[3] = 0;
   out_2058932946514885206[4] = 0;
   out_2058932946514885206[5] = 0;
   out_2058932946514885206[6] = 0;
   out_2058932946514885206[7] = 1;
   out_2058932946514885206[8] = 0;
}
void h_27(double *state, double *unused, double *out_9156976732101439785) {
   out_9156976732101439785[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2026140019065994734) {
   out_2026140019065994734[0] = 0;
   out_2026140019065994734[1] = 0;
   out_2026140019065994734[2] = 0;
   out_2026140019065994734[3] = 1;
   out_2026140019065994734[4] = 0;
   out_2026140019065994734[5] = 0;
   out_2026140019065994734[6] = 0;
   out_2026140019065994734[7] = 0;
   out_2026140019065994734[8] = 0;
}
void h_29(double *state, double *unused, double *out_152050056924260649) {
   out_152050056924260649[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4711134675180811829) {
   out_4711134675180811829[0] = 0;
   out_4711134675180811829[1] = 1;
   out_4711134675180811829[2] = 0;
   out_4711134675180811829[3] = 0;
   out_4711134675180811829[4] = 0;
   out_4711134675180811829[5] = 0;
   out_4711134675180811829[6] = 0;
   out_4711134675180811829[7] = 0;
   out_4711134675180811829[8] = 0;
}
void h_28(double *state, double *unused, double *out_5008203500364451323) {
   out_5008203500364451323[0] = state[0];
}
void H_28(double *state, double *unused, double *out_371264341888718745) {
   out_371264341888718745[0] = 1;
   out_371264341888718745[1] = 0;
   out_371264341888718745[2] = 0;
   out_371264341888718745[3] = 0;
   out_371264341888718745[4] = 0;
   out_371264341888718745[5] = 0;
   out_371264341888718745[6] = 0;
   out_371264341888718745[7] = 0;
   out_371264341888718745[8] = 0;
}
void h_31(double *state, double *unused, double *out_650644545693243836) {
   out_650644545693243836[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2685141048748236682) {
   out_2685141048748236682[0] = 0;
   out_2685141048748236682[1] = 0;
   out_2685141048748236682[2] = 0;
   out_2685141048748236682[3] = 0;
   out_2685141048748236682[4] = 0;
   out_2685141048748236682[5] = 0;
   out_2685141048748236682[6] = 0;
   out_2685141048748236682[7] = 0;
   out_2685141048748236682[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6969881541203456864) {
  err_fun(nom_x, delta_x, out_6969881541203456864);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1550632269520691561) {
  inv_err_fun(nom_x, true_x, out_1550632269520691561);
}
void car_H_mod_fun(double *state, double *out_4664115282963326508) {
  H_mod_fun(state, out_4664115282963326508);
}
void car_f_fun(double *state, double dt, double *out_3600870033497684324) {
  f_fun(state,  dt, out_3600870033497684324);
}
void car_F_fun(double *state, double dt, double *out_2816270813315995545) {
  F_fun(state,  dt, out_2816270813315995545);
}
void car_h_25(double *state, double *unused, double *out_751319912457213132) {
  h_25(state, unused, out_751319912457213132);
}
void car_H_25(double *state, double *unused, double *out_1682570372359171018) {
  H_25(state, unused, out_1682570372359171018);
}
void car_h_24(double *state, double *unused, double *out_8433912697962430457) {
  h_24(state, unused, out_8433912697962430457);
}
void car_H_24(double *state, double *unused, double *out_8916819690476358536) {
  H_24(state, unused, out_8916819690476358536);
}
void car_h_30(double *state, double *unused, double *out_4849623210736584996) {
  h_30(state, unused, out_4849623210736584996);
}
void car_H_30(double *state, double *unused, double *out_4200903330866419645) {
  H_30(state, unused, out_4200903330866419645);
}
void car_h_26(double *state, double *unused, double *out_2228954837399959704) {
  h_26(state, unused, out_2228954837399959704);
}
void car_H_26(double *state, double *unused, double *out_2058932946514885206) {
  H_26(state, unused, out_2058932946514885206);
}
void car_h_27(double *state, double *unused, double *out_9156976732101439785) {
  h_27(state, unused, out_9156976732101439785);
}
void car_H_27(double *state, double *unused, double *out_2026140019065994734) {
  H_27(state, unused, out_2026140019065994734);
}
void car_h_29(double *state, double *unused, double *out_152050056924260649) {
  h_29(state, unused, out_152050056924260649);
}
void car_H_29(double *state, double *unused, double *out_4711134675180811829) {
  H_29(state, unused, out_4711134675180811829);
}
void car_h_28(double *state, double *unused, double *out_5008203500364451323) {
  h_28(state, unused, out_5008203500364451323);
}
void car_H_28(double *state, double *unused, double *out_371264341888718745) {
  H_28(state, unused, out_371264341888718745);
}
void car_h_31(double *state, double *unused, double *out_650644545693243836) {
  h_31(state, unused, out_650644545693243836);
}
void car_H_31(double *state, double *unused, double *out_2685141048748236682) {
  H_31(state, unused, out_2685141048748236682);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
