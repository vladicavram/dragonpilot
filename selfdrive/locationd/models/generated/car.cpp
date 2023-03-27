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
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5160944041548397052) {
   out_5160944041548397052[0] = delta_x[0] + nom_x[0];
   out_5160944041548397052[1] = delta_x[1] + nom_x[1];
   out_5160944041548397052[2] = delta_x[2] + nom_x[2];
   out_5160944041548397052[3] = delta_x[3] + nom_x[3];
   out_5160944041548397052[4] = delta_x[4] + nom_x[4];
   out_5160944041548397052[5] = delta_x[5] + nom_x[5];
   out_5160944041548397052[6] = delta_x[6] + nom_x[6];
   out_5160944041548397052[7] = delta_x[7] + nom_x[7];
   out_5160944041548397052[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2969723540163324172) {
   out_2969723540163324172[0] = -nom_x[0] + true_x[0];
   out_2969723540163324172[1] = -nom_x[1] + true_x[1];
   out_2969723540163324172[2] = -nom_x[2] + true_x[2];
   out_2969723540163324172[3] = -nom_x[3] + true_x[3];
   out_2969723540163324172[4] = -nom_x[4] + true_x[4];
   out_2969723540163324172[5] = -nom_x[5] + true_x[5];
   out_2969723540163324172[6] = -nom_x[6] + true_x[6];
   out_2969723540163324172[7] = -nom_x[7] + true_x[7];
   out_2969723540163324172[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2213511003102344286) {
   out_2213511003102344286[0] = 1.0;
   out_2213511003102344286[1] = 0;
   out_2213511003102344286[2] = 0;
   out_2213511003102344286[3] = 0;
   out_2213511003102344286[4] = 0;
   out_2213511003102344286[5] = 0;
   out_2213511003102344286[6] = 0;
   out_2213511003102344286[7] = 0;
   out_2213511003102344286[8] = 0;
   out_2213511003102344286[9] = 0;
   out_2213511003102344286[10] = 1.0;
   out_2213511003102344286[11] = 0;
   out_2213511003102344286[12] = 0;
   out_2213511003102344286[13] = 0;
   out_2213511003102344286[14] = 0;
   out_2213511003102344286[15] = 0;
   out_2213511003102344286[16] = 0;
   out_2213511003102344286[17] = 0;
   out_2213511003102344286[18] = 0;
   out_2213511003102344286[19] = 0;
   out_2213511003102344286[20] = 1.0;
   out_2213511003102344286[21] = 0;
   out_2213511003102344286[22] = 0;
   out_2213511003102344286[23] = 0;
   out_2213511003102344286[24] = 0;
   out_2213511003102344286[25] = 0;
   out_2213511003102344286[26] = 0;
   out_2213511003102344286[27] = 0;
   out_2213511003102344286[28] = 0;
   out_2213511003102344286[29] = 0;
   out_2213511003102344286[30] = 1.0;
   out_2213511003102344286[31] = 0;
   out_2213511003102344286[32] = 0;
   out_2213511003102344286[33] = 0;
   out_2213511003102344286[34] = 0;
   out_2213511003102344286[35] = 0;
   out_2213511003102344286[36] = 0;
   out_2213511003102344286[37] = 0;
   out_2213511003102344286[38] = 0;
   out_2213511003102344286[39] = 0;
   out_2213511003102344286[40] = 1.0;
   out_2213511003102344286[41] = 0;
   out_2213511003102344286[42] = 0;
   out_2213511003102344286[43] = 0;
   out_2213511003102344286[44] = 0;
   out_2213511003102344286[45] = 0;
   out_2213511003102344286[46] = 0;
   out_2213511003102344286[47] = 0;
   out_2213511003102344286[48] = 0;
   out_2213511003102344286[49] = 0;
   out_2213511003102344286[50] = 1.0;
   out_2213511003102344286[51] = 0;
   out_2213511003102344286[52] = 0;
   out_2213511003102344286[53] = 0;
   out_2213511003102344286[54] = 0;
   out_2213511003102344286[55] = 0;
   out_2213511003102344286[56] = 0;
   out_2213511003102344286[57] = 0;
   out_2213511003102344286[58] = 0;
   out_2213511003102344286[59] = 0;
   out_2213511003102344286[60] = 1.0;
   out_2213511003102344286[61] = 0;
   out_2213511003102344286[62] = 0;
   out_2213511003102344286[63] = 0;
   out_2213511003102344286[64] = 0;
   out_2213511003102344286[65] = 0;
   out_2213511003102344286[66] = 0;
   out_2213511003102344286[67] = 0;
   out_2213511003102344286[68] = 0;
   out_2213511003102344286[69] = 0;
   out_2213511003102344286[70] = 1.0;
   out_2213511003102344286[71] = 0;
   out_2213511003102344286[72] = 0;
   out_2213511003102344286[73] = 0;
   out_2213511003102344286[74] = 0;
   out_2213511003102344286[75] = 0;
   out_2213511003102344286[76] = 0;
   out_2213511003102344286[77] = 0;
   out_2213511003102344286[78] = 0;
   out_2213511003102344286[79] = 0;
   out_2213511003102344286[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8059637990431312058) {
   out_8059637990431312058[0] = state[0];
   out_8059637990431312058[1] = state[1];
   out_8059637990431312058[2] = state[2];
   out_8059637990431312058[3] = state[3];
   out_8059637990431312058[4] = state[4];
   out_8059637990431312058[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8059637990431312058[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8059637990431312058[7] = state[7];
   out_8059637990431312058[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7046649207988276170) {
   out_7046649207988276170[0] = 1;
   out_7046649207988276170[1] = 0;
   out_7046649207988276170[2] = 0;
   out_7046649207988276170[3] = 0;
   out_7046649207988276170[4] = 0;
   out_7046649207988276170[5] = 0;
   out_7046649207988276170[6] = 0;
   out_7046649207988276170[7] = 0;
   out_7046649207988276170[8] = 0;
   out_7046649207988276170[9] = 0;
   out_7046649207988276170[10] = 1;
   out_7046649207988276170[11] = 0;
   out_7046649207988276170[12] = 0;
   out_7046649207988276170[13] = 0;
   out_7046649207988276170[14] = 0;
   out_7046649207988276170[15] = 0;
   out_7046649207988276170[16] = 0;
   out_7046649207988276170[17] = 0;
   out_7046649207988276170[18] = 0;
   out_7046649207988276170[19] = 0;
   out_7046649207988276170[20] = 1;
   out_7046649207988276170[21] = 0;
   out_7046649207988276170[22] = 0;
   out_7046649207988276170[23] = 0;
   out_7046649207988276170[24] = 0;
   out_7046649207988276170[25] = 0;
   out_7046649207988276170[26] = 0;
   out_7046649207988276170[27] = 0;
   out_7046649207988276170[28] = 0;
   out_7046649207988276170[29] = 0;
   out_7046649207988276170[30] = 1;
   out_7046649207988276170[31] = 0;
   out_7046649207988276170[32] = 0;
   out_7046649207988276170[33] = 0;
   out_7046649207988276170[34] = 0;
   out_7046649207988276170[35] = 0;
   out_7046649207988276170[36] = 0;
   out_7046649207988276170[37] = 0;
   out_7046649207988276170[38] = 0;
   out_7046649207988276170[39] = 0;
   out_7046649207988276170[40] = 1;
   out_7046649207988276170[41] = 0;
   out_7046649207988276170[42] = 0;
   out_7046649207988276170[43] = 0;
   out_7046649207988276170[44] = 0;
   out_7046649207988276170[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7046649207988276170[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7046649207988276170[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7046649207988276170[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7046649207988276170[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7046649207988276170[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7046649207988276170[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7046649207988276170[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7046649207988276170[53] = -9.8000000000000007*dt;
   out_7046649207988276170[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7046649207988276170[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7046649207988276170[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7046649207988276170[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7046649207988276170[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7046649207988276170[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7046649207988276170[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7046649207988276170[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7046649207988276170[62] = 0;
   out_7046649207988276170[63] = 0;
   out_7046649207988276170[64] = 0;
   out_7046649207988276170[65] = 0;
   out_7046649207988276170[66] = 0;
   out_7046649207988276170[67] = 0;
   out_7046649207988276170[68] = 0;
   out_7046649207988276170[69] = 0;
   out_7046649207988276170[70] = 1;
   out_7046649207988276170[71] = 0;
   out_7046649207988276170[72] = 0;
   out_7046649207988276170[73] = 0;
   out_7046649207988276170[74] = 0;
   out_7046649207988276170[75] = 0;
   out_7046649207988276170[76] = 0;
   out_7046649207988276170[77] = 0;
   out_7046649207988276170[78] = 0;
   out_7046649207988276170[79] = 0;
   out_7046649207988276170[80] = 1;
}
void h_25(double *state, double *unused, double *out_6666103549862703374) {
   out_6666103549862703374[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1119660535307403489) {
   out_1119660535307403489[0] = 0;
   out_1119660535307403489[1] = 0;
   out_1119660535307403489[2] = 0;
   out_1119660535307403489[3] = 0;
   out_1119660535307403489[4] = 0;
   out_1119660535307403489[5] = 0;
   out_1119660535307403489[6] = 1;
   out_1119660535307403489[7] = 0;
   out_1119660535307403489[8] = 0;
}
void h_24(double *state, double *unused, double *out_3197946087798995724) {
   out_3197946087798995724[0] = state[4];
   out_3197946087798995724[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8108404650761791736) {
   out_8108404650761791736[0] = 0;
   out_8108404650761791736[1] = 0;
   out_8108404650761791736[2] = 0;
   out_8108404650761791736[3] = 0;
   out_8108404650761791736[4] = 1;
   out_8108404650761791736[5] = 0;
   out_8108404650761791736[6] = 0;
   out_8108404650761791736[7] = 0;
   out_8108404650761791736[8] = 0;
   out_8108404650761791736[9] = 0;
   out_8108404650761791736[10] = 0;
   out_8108404650761791736[11] = 0;
   out_8108404650761791736[12] = 0;
   out_8108404650761791736[13] = 0;
   out_8108404650761791736[14] = 1;
   out_8108404650761791736[15] = 0;
   out_8108404650761791736[16] = 0;
   out_8108404650761791736[17] = 0;
}
void h_30(double *state, double *unused, double *out_6941297612147209263) {
   out_6941297612147209263[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1248999482450643559) {
   out_1248999482450643559[0] = 0;
   out_1248999482450643559[1] = 0;
   out_1248999482450643559[2] = 0;
   out_1248999482450643559[3] = 0;
   out_1248999482450643559[4] = 1;
   out_1248999482450643559[5] = 0;
   out_1248999482450643559[6] = 0;
   out_1248999482450643559[7] = 0;
   out_1248999482450643559[8] = 0;
}
void h_26(double *state, double *unused, double *out_6543066463881049121) {
   out_6543066463881049121[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4861163854181459713) {
   out_4861163854181459713[0] = 0;
   out_4861163854181459713[1] = 0;
   out_4861163854181459713[2] = 0;
   out_4861163854181459713[3] = 0;
   out_4861163854181459713[4] = 0;
   out_4861163854181459713[5] = 0;
   out_4861163854181459713[6] = 0;
   out_4861163854181459713[7] = 1;
   out_4861163854181459713[8] = 0;
}
void h_27(double *state, double *unused, double *out_336659153930362405) {
   out_336659153930362405[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3423762794251068470) {
   out_3423762794251068470[0] = 0;
   out_3423762794251068470[1] = 0;
   out_3423762794251068470[2] = 0;
   out_3423762794251068470[3] = 1;
   out_3423762794251068470[4] = 0;
   out_3423762794251068470[5] = 0;
   out_3423762794251068470[6] = 0;
   out_3423762794251068470[7] = 0;
   out_3423762794251068470[8] = 0;
}
void h_29(double *state, double *unused, double *out_7544613496823686780) {
   out_7544613496823686780[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5137125521120619503) {
   out_5137125521120619503[0] = 0;
   out_5137125521120619503[1] = 1;
   out_5137125521120619503[2] = 0;
   out_5137125521120619503[3] = 0;
   out_5137125521120619503[4] = 0;
   out_5137125521120619503[5] = 0;
   out_5137125521120619503[6] = 0;
   out_5137125521120619503[7] = 0;
   out_5137125521120619503[8] = 0;
}
void h_28(double *state, double *unused, double *out_8469415090921683557) {
   out_8469415090921683557[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8227219535519401539) {
   out_8227219535519401539[0] = 1;
   out_8227219535519401539[1] = 0;
   out_8227219535519401539[2] = 0;
   out_8227219535519401539[3] = 0;
   out_8227219535519401539[4] = 0;
   out_8227219535519401539[5] = 0;
   out_8227219535519401539[6] = 0;
   out_8227219535519401539[7] = 0;
   out_8227219535519401539[8] = 0;
}
void h_31(double *state, double *unused, double *out_3887701028962324883) {
   out_3887701028962324883[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1089014573430443061) {
   out_1089014573430443061[0] = 0;
   out_1089014573430443061[1] = 0;
   out_1089014573430443061[2] = 0;
   out_1089014573430443061[3] = 0;
   out_1089014573430443061[4] = 0;
   out_1089014573430443061[5] = 0;
   out_1089014573430443061[6] = 0;
   out_1089014573430443061[7] = 0;
   out_1089014573430443061[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5160944041548397052) {
  err_fun(nom_x, delta_x, out_5160944041548397052);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2969723540163324172) {
  inv_err_fun(nom_x, true_x, out_2969723540163324172);
}
void car_H_mod_fun(double *state, double *out_2213511003102344286) {
  H_mod_fun(state, out_2213511003102344286);
}
void car_f_fun(double *state, double dt, double *out_8059637990431312058) {
  f_fun(state,  dt, out_8059637990431312058);
}
void car_F_fun(double *state, double dt, double *out_7046649207988276170) {
  F_fun(state,  dt, out_7046649207988276170);
}
void car_h_25(double *state, double *unused, double *out_6666103549862703374) {
  h_25(state, unused, out_6666103549862703374);
}
void car_H_25(double *state, double *unused, double *out_1119660535307403489) {
  H_25(state, unused, out_1119660535307403489);
}
void car_h_24(double *state, double *unused, double *out_3197946087798995724) {
  h_24(state, unused, out_3197946087798995724);
}
void car_H_24(double *state, double *unused, double *out_8108404650761791736) {
  H_24(state, unused, out_8108404650761791736);
}
void car_h_30(double *state, double *unused, double *out_6941297612147209263) {
  h_30(state, unused, out_6941297612147209263);
}
void car_H_30(double *state, double *unused, double *out_1248999482450643559) {
  H_30(state, unused, out_1248999482450643559);
}
void car_h_26(double *state, double *unused, double *out_6543066463881049121) {
  h_26(state, unused, out_6543066463881049121);
}
void car_H_26(double *state, double *unused, double *out_4861163854181459713) {
  H_26(state, unused, out_4861163854181459713);
}
void car_h_27(double *state, double *unused, double *out_336659153930362405) {
  h_27(state, unused, out_336659153930362405);
}
void car_H_27(double *state, double *unused, double *out_3423762794251068470) {
  H_27(state, unused, out_3423762794251068470);
}
void car_h_29(double *state, double *unused, double *out_7544613496823686780) {
  h_29(state, unused, out_7544613496823686780);
}
void car_H_29(double *state, double *unused, double *out_5137125521120619503) {
  H_29(state, unused, out_5137125521120619503);
}
void car_h_28(double *state, double *unused, double *out_8469415090921683557) {
  h_28(state, unused, out_8469415090921683557);
}
void car_H_28(double *state, double *unused, double *out_8227219535519401539) {
  H_28(state, unused, out_8227219535519401539);
}
void car_h_31(double *state, double *unused, double *out_3887701028962324883) {
  h_31(state, unused, out_3887701028962324883);
}
void car_H_31(double *state, double *unused, double *out_1089014573430443061) {
  H_31(state, unused, out_1089014573430443061);
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

ekf_init(car);
