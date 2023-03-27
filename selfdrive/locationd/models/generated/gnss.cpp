#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_324237608516468626) {
   out_324237608516468626[0] = delta_x[0] + nom_x[0];
   out_324237608516468626[1] = delta_x[1] + nom_x[1];
   out_324237608516468626[2] = delta_x[2] + nom_x[2];
   out_324237608516468626[3] = delta_x[3] + nom_x[3];
   out_324237608516468626[4] = delta_x[4] + nom_x[4];
   out_324237608516468626[5] = delta_x[5] + nom_x[5];
   out_324237608516468626[6] = delta_x[6] + nom_x[6];
   out_324237608516468626[7] = delta_x[7] + nom_x[7];
   out_324237608516468626[8] = delta_x[8] + nom_x[8];
   out_324237608516468626[9] = delta_x[9] + nom_x[9];
   out_324237608516468626[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8569264215754462263) {
   out_8569264215754462263[0] = -nom_x[0] + true_x[0];
   out_8569264215754462263[1] = -nom_x[1] + true_x[1];
   out_8569264215754462263[2] = -nom_x[2] + true_x[2];
   out_8569264215754462263[3] = -nom_x[3] + true_x[3];
   out_8569264215754462263[4] = -nom_x[4] + true_x[4];
   out_8569264215754462263[5] = -nom_x[5] + true_x[5];
   out_8569264215754462263[6] = -nom_x[6] + true_x[6];
   out_8569264215754462263[7] = -nom_x[7] + true_x[7];
   out_8569264215754462263[8] = -nom_x[8] + true_x[8];
   out_8569264215754462263[9] = -nom_x[9] + true_x[9];
   out_8569264215754462263[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4370407027249388340) {
   out_4370407027249388340[0] = 1.0;
   out_4370407027249388340[1] = 0;
   out_4370407027249388340[2] = 0;
   out_4370407027249388340[3] = 0;
   out_4370407027249388340[4] = 0;
   out_4370407027249388340[5] = 0;
   out_4370407027249388340[6] = 0;
   out_4370407027249388340[7] = 0;
   out_4370407027249388340[8] = 0;
   out_4370407027249388340[9] = 0;
   out_4370407027249388340[10] = 0;
   out_4370407027249388340[11] = 0;
   out_4370407027249388340[12] = 1.0;
   out_4370407027249388340[13] = 0;
   out_4370407027249388340[14] = 0;
   out_4370407027249388340[15] = 0;
   out_4370407027249388340[16] = 0;
   out_4370407027249388340[17] = 0;
   out_4370407027249388340[18] = 0;
   out_4370407027249388340[19] = 0;
   out_4370407027249388340[20] = 0;
   out_4370407027249388340[21] = 0;
   out_4370407027249388340[22] = 0;
   out_4370407027249388340[23] = 0;
   out_4370407027249388340[24] = 1.0;
   out_4370407027249388340[25] = 0;
   out_4370407027249388340[26] = 0;
   out_4370407027249388340[27] = 0;
   out_4370407027249388340[28] = 0;
   out_4370407027249388340[29] = 0;
   out_4370407027249388340[30] = 0;
   out_4370407027249388340[31] = 0;
   out_4370407027249388340[32] = 0;
   out_4370407027249388340[33] = 0;
   out_4370407027249388340[34] = 0;
   out_4370407027249388340[35] = 0;
   out_4370407027249388340[36] = 1.0;
   out_4370407027249388340[37] = 0;
   out_4370407027249388340[38] = 0;
   out_4370407027249388340[39] = 0;
   out_4370407027249388340[40] = 0;
   out_4370407027249388340[41] = 0;
   out_4370407027249388340[42] = 0;
   out_4370407027249388340[43] = 0;
   out_4370407027249388340[44] = 0;
   out_4370407027249388340[45] = 0;
   out_4370407027249388340[46] = 0;
   out_4370407027249388340[47] = 0;
   out_4370407027249388340[48] = 1.0;
   out_4370407027249388340[49] = 0;
   out_4370407027249388340[50] = 0;
   out_4370407027249388340[51] = 0;
   out_4370407027249388340[52] = 0;
   out_4370407027249388340[53] = 0;
   out_4370407027249388340[54] = 0;
   out_4370407027249388340[55] = 0;
   out_4370407027249388340[56] = 0;
   out_4370407027249388340[57] = 0;
   out_4370407027249388340[58] = 0;
   out_4370407027249388340[59] = 0;
   out_4370407027249388340[60] = 1.0;
   out_4370407027249388340[61] = 0;
   out_4370407027249388340[62] = 0;
   out_4370407027249388340[63] = 0;
   out_4370407027249388340[64] = 0;
   out_4370407027249388340[65] = 0;
   out_4370407027249388340[66] = 0;
   out_4370407027249388340[67] = 0;
   out_4370407027249388340[68] = 0;
   out_4370407027249388340[69] = 0;
   out_4370407027249388340[70] = 0;
   out_4370407027249388340[71] = 0;
   out_4370407027249388340[72] = 1.0;
   out_4370407027249388340[73] = 0;
   out_4370407027249388340[74] = 0;
   out_4370407027249388340[75] = 0;
   out_4370407027249388340[76] = 0;
   out_4370407027249388340[77] = 0;
   out_4370407027249388340[78] = 0;
   out_4370407027249388340[79] = 0;
   out_4370407027249388340[80] = 0;
   out_4370407027249388340[81] = 0;
   out_4370407027249388340[82] = 0;
   out_4370407027249388340[83] = 0;
   out_4370407027249388340[84] = 1.0;
   out_4370407027249388340[85] = 0;
   out_4370407027249388340[86] = 0;
   out_4370407027249388340[87] = 0;
   out_4370407027249388340[88] = 0;
   out_4370407027249388340[89] = 0;
   out_4370407027249388340[90] = 0;
   out_4370407027249388340[91] = 0;
   out_4370407027249388340[92] = 0;
   out_4370407027249388340[93] = 0;
   out_4370407027249388340[94] = 0;
   out_4370407027249388340[95] = 0;
   out_4370407027249388340[96] = 1.0;
   out_4370407027249388340[97] = 0;
   out_4370407027249388340[98] = 0;
   out_4370407027249388340[99] = 0;
   out_4370407027249388340[100] = 0;
   out_4370407027249388340[101] = 0;
   out_4370407027249388340[102] = 0;
   out_4370407027249388340[103] = 0;
   out_4370407027249388340[104] = 0;
   out_4370407027249388340[105] = 0;
   out_4370407027249388340[106] = 0;
   out_4370407027249388340[107] = 0;
   out_4370407027249388340[108] = 1.0;
   out_4370407027249388340[109] = 0;
   out_4370407027249388340[110] = 0;
   out_4370407027249388340[111] = 0;
   out_4370407027249388340[112] = 0;
   out_4370407027249388340[113] = 0;
   out_4370407027249388340[114] = 0;
   out_4370407027249388340[115] = 0;
   out_4370407027249388340[116] = 0;
   out_4370407027249388340[117] = 0;
   out_4370407027249388340[118] = 0;
   out_4370407027249388340[119] = 0;
   out_4370407027249388340[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_4386081775298855099) {
   out_4386081775298855099[0] = dt*state[3] + state[0];
   out_4386081775298855099[1] = dt*state[4] + state[1];
   out_4386081775298855099[2] = dt*state[5] + state[2];
   out_4386081775298855099[3] = state[3];
   out_4386081775298855099[4] = state[4];
   out_4386081775298855099[5] = state[5];
   out_4386081775298855099[6] = dt*state[7] + state[6];
   out_4386081775298855099[7] = dt*state[8] + state[7];
   out_4386081775298855099[8] = state[8];
   out_4386081775298855099[9] = state[9];
   out_4386081775298855099[10] = state[10];
}
void F_fun(double *state, double dt, double *out_2273348682544228066) {
   out_2273348682544228066[0] = 1;
   out_2273348682544228066[1] = 0;
   out_2273348682544228066[2] = 0;
   out_2273348682544228066[3] = dt;
   out_2273348682544228066[4] = 0;
   out_2273348682544228066[5] = 0;
   out_2273348682544228066[6] = 0;
   out_2273348682544228066[7] = 0;
   out_2273348682544228066[8] = 0;
   out_2273348682544228066[9] = 0;
   out_2273348682544228066[10] = 0;
   out_2273348682544228066[11] = 0;
   out_2273348682544228066[12] = 1;
   out_2273348682544228066[13] = 0;
   out_2273348682544228066[14] = 0;
   out_2273348682544228066[15] = dt;
   out_2273348682544228066[16] = 0;
   out_2273348682544228066[17] = 0;
   out_2273348682544228066[18] = 0;
   out_2273348682544228066[19] = 0;
   out_2273348682544228066[20] = 0;
   out_2273348682544228066[21] = 0;
   out_2273348682544228066[22] = 0;
   out_2273348682544228066[23] = 0;
   out_2273348682544228066[24] = 1;
   out_2273348682544228066[25] = 0;
   out_2273348682544228066[26] = 0;
   out_2273348682544228066[27] = dt;
   out_2273348682544228066[28] = 0;
   out_2273348682544228066[29] = 0;
   out_2273348682544228066[30] = 0;
   out_2273348682544228066[31] = 0;
   out_2273348682544228066[32] = 0;
   out_2273348682544228066[33] = 0;
   out_2273348682544228066[34] = 0;
   out_2273348682544228066[35] = 0;
   out_2273348682544228066[36] = 1;
   out_2273348682544228066[37] = 0;
   out_2273348682544228066[38] = 0;
   out_2273348682544228066[39] = 0;
   out_2273348682544228066[40] = 0;
   out_2273348682544228066[41] = 0;
   out_2273348682544228066[42] = 0;
   out_2273348682544228066[43] = 0;
   out_2273348682544228066[44] = 0;
   out_2273348682544228066[45] = 0;
   out_2273348682544228066[46] = 0;
   out_2273348682544228066[47] = 0;
   out_2273348682544228066[48] = 1;
   out_2273348682544228066[49] = 0;
   out_2273348682544228066[50] = 0;
   out_2273348682544228066[51] = 0;
   out_2273348682544228066[52] = 0;
   out_2273348682544228066[53] = 0;
   out_2273348682544228066[54] = 0;
   out_2273348682544228066[55] = 0;
   out_2273348682544228066[56] = 0;
   out_2273348682544228066[57] = 0;
   out_2273348682544228066[58] = 0;
   out_2273348682544228066[59] = 0;
   out_2273348682544228066[60] = 1;
   out_2273348682544228066[61] = 0;
   out_2273348682544228066[62] = 0;
   out_2273348682544228066[63] = 0;
   out_2273348682544228066[64] = 0;
   out_2273348682544228066[65] = 0;
   out_2273348682544228066[66] = 0;
   out_2273348682544228066[67] = 0;
   out_2273348682544228066[68] = 0;
   out_2273348682544228066[69] = 0;
   out_2273348682544228066[70] = 0;
   out_2273348682544228066[71] = 0;
   out_2273348682544228066[72] = 1;
   out_2273348682544228066[73] = dt;
   out_2273348682544228066[74] = 0;
   out_2273348682544228066[75] = 0;
   out_2273348682544228066[76] = 0;
   out_2273348682544228066[77] = 0;
   out_2273348682544228066[78] = 0;
   out_2273348682544228066[79] = 0;
   out_2273348682544228066[80] = 0;
   out_2273348682544228066[81] = 0;
   out_2273348682544228066[82] = 0;
   out_2273348682544228066[83] = 0;
   out_2273348682544228066[84] = 1;
   out_2273348682544228066[85] = dt;
   out_2273348682544228066[86] = 0;
   out_2273348682544228066[87] = 0;
   out_2273348682544228066[88] = 0;
   out_2273348682544228066[89] = 0;
   out_2273348682544228066[90] = 0;
   out_2273348682544228066[91] = 0;
   out_2273348682544228066[92] = 0;
   out_2273348682544228066[93] = 0;
   out_2273348682544228066[94] = 0;
   out_2273348682544228066[95] = 0;
   out_2273348682544228066[96] = 1;
   out_2273348682544228066[97] = 0;
   out_2273348682544228066[98] = 0;
   out_2273348682544228066[99] = 0;
   out_2273348682544228066[100] = 0;
   out_2273348682544228066[101] = 0;
   out_2273348682544228066[102] = 0;
   out_2273348682544228066[103] = 0;
   out_2273348682544228066[104] = 0;
   out_2273348682544228066[105] = 0;
   out_2273348682544228066[106] = 0;
   out_2273348682544228066[107] = 0;
   out_2273348682544228066[108] = 1;
   out_2273348682544228066[109] = 0;
   out_2273348682544228066[110] = 0;
   out_2273348682544228066[111] = 0;
   out_2273348682544228066[112] = 0;
   out_2273348682544228066[113] = 0;
   out_2273348682544228066[114] = 0;
   out_2273348682544228066[115] = 0;
   out_2273348682544228066[116] = 0;
   out_2273348682544228066[117] = 0;
   out_2273348682544228066[118] = 0;
   out_2273348682544228066[119] = 0;
   out_2273348682544228066[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_2414446894350139589) {
   out_2414446894350139589[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_5974652309776416193) {
   out_5974652309776416193[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5974652309776416193[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5974652309776416193[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5974652309776416193[3] = 0;
   out_5974652309776416193[4] = 0;
   out_5974652309776416193[5] = 0;
   out_5974652309776416193[6] = 1;
   out_5974652309776416193[7] = 0;
   out_5974652309776416193[8] = 0;
   out_5974652309776416193[9] = 0;
   out_5974652309776416193[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_5821521653382466763) {
   out_5821521653382466763[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_7734522565204147379) {
   out_7734522565204147379[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7734522565204147379[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7734522565204147379[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7734522565204147379[3] = 0;
   out_7734522565204147379[4] = 0;
   out_7734522565204147379[5] = 0;
   out_7734522565204147379[6] = 1;
   out_7734522565204147379[7] = 0;
   out_7734522565204147379[8] = 0;
   out_7734522565204147379[9] = 1;
   out_7734522565204147379[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_1507807895002186558) {
   out_1507807895002186558[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_5264854063588760803) {
   out_5264854063588760803[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[6] = 0;
   out_5264854063588760803[7] = 1;
   out_5264854063588760803[8] = 0;
   out_5264854063588760803[9] = 0;
   out_5264854063588760803[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_1507807895002186558) {
   out_1507807895002186558[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_5264854063588760803) {
   out_5264854063588760803[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5264854063588760803[6] = 0;
   out_5264854063588760803[7] = 1;
   out_5264854063588760803[8] = 0;
   out_5264854063588760803[9] = 0;
   out_5264854063588760803[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_324237608516468626) {
  err_fun(nom_x, delta_x, out_324237608516468626);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8569264215754462263) {
  inv_err_fun(nom_x, true_x, out_8569264215754462263);
}
void gnss_H_mod_fun(double *state, double *out_4370407027249388340) {
  H_mod_fun(state, out_4370407027249388340);
}
void gnss_f_fun(double *state, double dt, double *out_4386081775298855099) {
  f_fun(state,  dt, out_4386081775298855099);
}
void gnss_F_fun(double *state, double dt, double *out_2273348682544228066) {
  F_fun(state,  dt, out_2273348682544228066);
}
void gnss_h_6(double *state, double *sat_pos, double *out_2414446894350139589) {
  h_6(state, sat_pos, out_2414446894350139589);
}
void gnss_H_6(double *state, double *sat_pos, double *out_5974652309776416193) {
  H_6(state, sat_pos, out_5974652309776416193);
}
void gnss_h_20(double *state, double *sat_pos, double *out_5821521653382466763) {
  h_20(state, sat_pos, out_5821521653382466763);
}
void gnss_H_20(double *state, double *sat_pos, double *out_7734522565204147379) {
  H_20(state, sat_pos, out_7734522565204147379);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1507807895002186558) {
  h_7(state, sat_pos_vel, out_1507807895002186558);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5264854063588760803) {
  H_7(state, sat_pos_vel, out_5264854063588760803);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1507807895002186558) {
  h_21(state, sat_pos_vel, out_1507807895002186558);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5264854063588760803) {
  H_21(state, sat_pos_vel, out_5264854063588760803);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
