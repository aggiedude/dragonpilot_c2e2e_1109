#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2351947283659598660);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8539310053726918653);
void gnss_H_mod_fun(double *state, double *out_2843304307356815987);
void gnss_f_fun(double *state, double dt, double *out_774769938980785647);
void gnss_F_fun(double *state, double dt, double *out_4467768114936024642);
void gnss_h_6(double *state, double *sat_pos, double *out_8206867657584711155);
void gnss_H_6(double *state, double *sat_pos, double *out_2748205046550560630);
void gnss_h_20(double *state, double *sat_pos, double *out_3144072620339266513);
void gnss_H_20(double *state, double *sat_pos, double *out_8303647801952025828);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7730590468597126339);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1168235388167943403);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7730590468597126339);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1168235388167943403);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}