#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5160944041548397052);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2969723540163324172);
void car_H_mod_fun(double *state, double *out_2213511003102344286);
void car_f_fun(double *state, double dt, double *out_8059637990431312058);
void car_F_fun(double *state, double dt, double *out_7046649207988276170);
void car_h_25(double *state, double *unused, double *out_6666103549862703374);
void car_H_25(double *state, double *unused, double *out_1119660535307403489);
void car_h_24(double *state, double *unused, double *out_3197946087798995724);
void car_H_24(double *state, double *unused, double *out_8108404650761791736);
void car_h_30(double *state, double *unused, double *out_6941297612147209263);
void car_H_30(double *state, double *unused, double *out_1248999482450643559);
void car_h_26(double *state, double *unused, double *out_6543066463881049121);
void car_H_26(double *state, double *unused, double *out_4861163854181459713);
void car_h_27(double *state, double *unused, double *out_336659153930362405);
void car_H_27(double *state, double *unused, double *out_3423762794251068470);
void car_h_29(double *state, double *unused, double *out_7544613496823686780);
void car_H_29(double *state, double *unused, double *out_5137125521120619503);
void car_h_28(double *state, double *unused, double *out_8469415090921683557);
void car_H_28(double *state, double *unused, double *out_8227219535519401539);
void car_h_31(double *state, double *unused, double *out_3887701028962324883);
void car_H_31(double *state, double *unused, double *out_1089014573430443061);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}