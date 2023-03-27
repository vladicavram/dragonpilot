#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_324237608516468626);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8569264215754462263);
void gnss_H_mod_fun(double *state, double *out_4370407027249388340);
void gnss_f_fun(double *state, double dt, double *out_4386081775298855099);
void gnss_F_fun(double *state, double dt, double *out_2273348682544228066);
void gnss_h_6(double *state, double *sat_pos, double *out_2414446894350139589);
void gnss_H_6(double *state, double *sat_pos, double *out_5974652309776416193);
void gnss_h_20(double *state, double *sat_pos, double *out_5821521653382466763);
void gnss_H_20(double *state, double *sat_pos, double *out_7734522565204147379);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_1507807895002186558);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5264854063588760803);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_1507807895002186558);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5264854063588760803);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}