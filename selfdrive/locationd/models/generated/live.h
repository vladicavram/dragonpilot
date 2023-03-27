#pragma once
#include "rednose/helpers/common_ekf.h"
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
void live_H(double *in_vec, double *out_3650258586047302313);
void live_err_fun(double *nom_x, double *delta_x, double *out_319164860628708381);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_61279048776004062);
void live_H_mod_fun(double *state, double *out_8884535280500598173);
void live_f_fun(double *state, double dt, double *out_1529620457729649381);
void live_F_fun(double *state, double dt, double *out_1152118045946151049);
void live_h_4(double *state, double *unused, double *out_1481217230938872647);
void live_H_4(double *state, double *unused, double *out_8962114299087111796);
void live_h_9(double *state, double *unused, double *out_2255850241051147115);
void live_H_9(double *state, double *unused, double *out_9203303945716702441);
void live_h_10(double *state, double *unused, double *out_3292043746715240724);
void live_H_10(double *state, double *unused, double *out_846922134723447269);
void live_h_12(double *state, double *unused, double *out_7159004250134852029);
void live_H_12(double *state, double *unused, double *out_4465173366590478025);
void live_h_35(double *state, double *unused, double *out_2632128346535043517);
void live_H_35(double *state, double *unused, double *out_6117967717249832444);
void live_h_32(double *state, double *unused, double *out_2865592043788577394);
void live_H_32(double *state, double *unused, double *out_5986802932309091975);
void live_h_13(double *state, double *unused, double *out_700089127386922507);
void live_H_13(double *state, double *unused, double *out_935496178521268066);
void live_h_14(double *state, double *unused, double *out_2255850241051147115);
void live_H_14(double *state, double *unused, double *out_9203303945716702441);
void live_h_33(double *state, double *unused, double *out_3195207143915802552);
void live_H_33(double *state, double *unused, double *out_2967410712610974840);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}