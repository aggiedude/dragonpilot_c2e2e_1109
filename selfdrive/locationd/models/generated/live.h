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
void live_H(double *in_vec, double *out_8687680363762493539);
void live_err_fun(double *nom_x, double *delta_x, double *out_8136490426412747881);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1524655933590429885);
void live_H_mod_fun(double *state, double *out_7453067886472422609);
void live_f_fun(double *state, double dt, double *out_3547963935686363060);
void live_F_fun(double *state, double dt, double *out_3254137990445528781);
void live_h_4(double *state, double *unused, double *out_4410787759120580449);
void live_H_4(double *state, double *unused, double *out_5512660098689845475);
void live_h_9(double *state, double *unused, double *out_4264504783842197092);
void live_H_9(double *state, double *unused, double *out_5271470452060254830);
void live_h_10(double *state, double *unused, double *out_1276705416788479855);
void live_H_10(double *state, double *unused, double *out_6589591562994162559);
void live_h_12(double *state, double *unused, double *out_4489758644873872208);
void live_H_12(double *state, double *unused, double *out_493203690657883680);
void live_h_35(double *state, double *unused, double *out_8696122944070004010);
void live_H_35(double *state, double *unused, double *out_2252359341667130029);
void live_h_32(double *state, double *unused, double *out_2592146495080937647);
void live_H_32(double *state, double *unused, double *out_8522971196901590084);
void live_h_13(double *state, double *unused, double *out_5320318911182793356);
void live_H_13(double *state, double *unused, double *out_6260235854627074248);
void live_h_14(double *state, double *unused, double *out_4264504783842197092);
void live_H_14(double *state, double *unused, double *out_5271470452060254830);
void live_h_33(double *state, double *unused, double *out_57093419937807770);
void live_H_33(double *state, double *unused, double *out_5402916346305987633);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}