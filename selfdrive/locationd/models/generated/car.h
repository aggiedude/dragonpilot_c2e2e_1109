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
void car_err_fun(double *nom_x, double *delta_x, double *out_1840749225442711100);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7887258947850824197);
void car_H_mod_fun(double *state, double *out_5849575757077882696);
void car_f_fun(double *state, double dt, double *out_1306455665935687231);
void car_F_fun(double *state, double dt, double *out_1015750461961158607);
void car_h_25(double *state, double *unused, double *out_579481060641732431);
void car_H_25(double *state, double *unused, double *out_8838814511200069080);
void car_h_24(double *state, double *unused, double *out_8628826922044058076);
void car_H_24(double *state, double *unused, double *out_7435279963503982970);
void car_h_30(double *state, double *unused, double *out_8060396416076809453);
void car_H_30(double *state, double *unused, double *out_6320481552692820453);
void car_h_26(double *state, double *unused, double *out_8663712300753286970);
void car_H_26(double *state, double *unused, double *out_5866426243635426312);
void car_h_27(double *state, double *unused, double *out_2233071042628120952);
void car_H_27(double *state, double *unused, double *out_4096887481508877236);
void car_h_29(double *state, double *unused, double *out_8528880318658063037);
void car_H_29(double *state, double *unused, double *out_5810250208378428269);
void car_h_28(double *state, double *unused, double *out_4837476363436818603);
void car_H_28(double *state, double *unused, double *out_7554094848261592773);
void car_h_31(double *state, double *unused, double *out_7350316286992083367);
void car_H_31(double *state, double *unused, double *out_5240218141402074836);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}