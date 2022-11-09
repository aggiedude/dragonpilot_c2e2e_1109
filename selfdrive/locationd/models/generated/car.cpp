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
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1840749225442711100) {
   out_1840749225442711100[0] = delta_x[0] + nom_x[0];
   out_1840749225442711100[1] = delta_x[1] + nom_x[1];
   out_1840749225442711100[2] = delta_x[2] + nom_x[2];
   out_1840749225442711100[3] = delta_x[3] + nom_x[3];
   out_1840749225442711100[4] = delta_x[4] + nom_x[4];
   out_1840749225442711100[5] = delta_x[5] + nom_x[5];
   out_1840749225442711100[6] = delta_x[6] + nom_x[6];
   out_1840749225442711100[7] = delta_x[7] + nom_x[7];
   out_1840749225442711100[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7887258947850824197) {
   out_7887258947850824197[0] = -nom_x[0] + true_x[0];
   out_7887258947850824197[1] = -nom_x[1] + true_x[1];
   out_7887258947850824197[2] = -nom_x[2] + true_x[2];
   out_7887258947850824197[3] = -nom_x[3] + true_x[3];
   out_7887258947850824197[4] = -nom_x[4] + true_x[4];
   out_7887258947850824197[5] = -nom_x[5] + true_x[5];
   out_7887258947850824197[6] = -nom_x[6] + true_x[6];
   out_7887258947850824197[7] = -nom_x[7] + true_x[7];
   out_7887258947850824197[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5849575757077882696) {
   out_5849575757077882696[0] = 1.0;
   out_5849575757077882696[1] = 0;
   out_5849575757077882696[2] = 0;
   out_5849575757077882696[3] = 0;
   out_5849575757077882696[4] = 0;
   out_5849575757077882696[5] = 0;
   out_5849575757077882696[6] = 0;
   out_5849575757077882696[7] = 0;
   out_5849575757077882696[8] = 0;
   out_5849575757077882696[9] = 0;
   out_5849575757077882696[10] = 1.0;
   out_5849575757077882696[11] = 0;
   out_5849575757077882696[12] = 0;
   out_5849575757077882696[13] = 0;
   out_5849575757077882696[14] = 0;
   out_5849575757077882696[15] = 0;
   out_5849575757077882696[16] = 0;
   out_5849575757077882696[17] = 0;
   out_5849575757077882696[18] = 0;
   out_5849575757077882696[19] = 0;
   out_5849575757077882696[20] = 1.0;
   out_5849575757077882696[21] = 0;
   out_5849575757077882696[22] = 0;
   out_5849575757077882696[23] = 0;
   out_5849575757077882696[24] = 0;
   out_5849575757077882696[25] = 0;
   out_5849575757077882696[26] = 0;
   out_5849575757077882696[27] = 0;
   out_5849575757077882696[28] = 0;
   out_5849575757077882696[29] = 0;
   out_5849575757077882696[30] = 1.0;
   out_5849575757077882696[31] = 0;
   out_5849575757077882696[32] = 0;
   out_5849575757077882696[33] = 0;
   out_5849575757077882696[34] = 0;
   out_5849575757077882696[35] = 0;
   out_5849575757077882696[36] = 0;
   out_5849575757077882696[37] = 0;
   out_5849575757077882696[38] = 0;
   out_5849575757077882696[39] = 0;
   out_5849575757077882696[40] = 1.0;
   out_5849575757077882696[41] = 0;
   out_5849575757077882696[42] = 0;
   out_5849575757077882696[43] = 0;
   out_5849575757077882696[44] = 0;
   out_5849575757077882696[45] = 0;
   out_5849575757077882696[46] = 0;
   out_5849575757077882696[47] = 0;
   out_5849575757077882696[48] = 0;
   out_5849575757077882696[49] = 0;
   out_5849575757077882696[50] = 1.0;
   out_5849575757077882696[51] = 0;
   out_5849575757077882696[52] = 0;
   out_5849575757077882696[53] = 0;
   out_5849575757077882696[54] = 0;
   out_5849575757077882696[55] = 0;
   out_5849575757077882696[56] = 0;
   out_5849575757077882696[57] = 0;
   out_5849575757077882696[58] = 0;
   out_5849575757077882696[59] = 0;
   out_5849575757077882696[60] = 1.0;
   out_5849575757077882696[61] = 0;
   out_5849575757077882696[62] = 0;
   out_5849575757077882696[63] = 0;
   out_5849575757077882696[64] = 0;
   out_5849575757077882696[65] = 0;
   out_5849575757077882696[66] = 0;
   out_5849575757077882696[67] = 0;
   out_5849575757077882696[68] = 0;
   out_5849575757077882696[69] = 0;
   out_5849575757077882696[70] = 1.0;
   out_5849575757077882696[71] = 0;
   out_5849575757077882696[72] = 0;
   out_5849575757077882696[73] = 0;
   out_5849575757077882696[74] = 0;
   out_5849575757077882696[75] = 0;
   out_5849575757077882696[76] = 0;
   out_5849575757077882696[77] = 0;
   out_5849575757077882696[78] = 0;
   out_5849575757077882696[79] = 0;
   out_5849575757077882696[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1306455665935687231) {
   out_1306455665935687231[0] = state[0];
   out_1306455665935687231[1] = state[1];
   out_1306455665935687231[2] = state[2];
   out_1306455665935687231[3] = state[3];
   out_1306455665935687231[4] = state[4];
   out_1306455665935687231[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1306455665935687231[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1306455665935687231[7] = state[7];
   out_1306455665935687231[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1015750461961158607) {
   out_1015750461961158607[0] = 1;
   out_1015750461961158607[1] = 0;
   out_1015750461961158607[2] = 0;
   out_1015750461961158607[3] = 0;
   out_1015750461961158607[4] = 0;
   out_1015750461961158607[5] = 0;
   out_1015750461961158607[6] = 0;
   out_1015750461961158607[7] = 0;
   out_1015750461961158607[8] = 0;
   out_1015750461961158607[9] = 0;
   out_1015750461961158607[10] = 1;
   out_1015750461961158607[11] = 0;
   out_1015750461961158607[12] = 0;
   out_1015750461961158607[13] = 0;
   out_1015750461961158607[14] = 0;
   out_1015750461961158607[15] = 0;
   out_1015750461961158607[16] = 0;
   out_1015750461961158607[17] = 0;
   out_1015750461961158607[18] = 0;
   out_1015750461961158607[19] = 0;
   out_1015750461961158607[20] = 1;
   out_1015750461961158607[21] = 0;
   out_1015750461961158607[22] = 0;
   out_1015750461961158607[23] = 0;
   out_1015750461961158607[24] = 0;
   out_1015750461961158607[25] = 0;
   out_1015750461961158607[26] = 0;
   out_1015750461961158607[27] = 0;
   out_1015750461961158607[28] = 0;
   out_1015750461961158607[29] = 0;
   out_1015750461961158607[30] = 1;
   out_1015750461961158607[31] = 0;
   out_1015750461961158607[32] = 0;
   out_1015750461961158607[33] = 0;
   out_1015750461961158607[34] = 0;
   out_1015750461961158607[35] = 0;
   out_1015750461961158607[36] = 0;
   out_1015750461961158607[37] = 0;
   out_1015750461961158607[38] = 0;
   out_1015750461961158607[39] = 0;
   out_1015750461961158607[40] = 1;
   out_1015750461961158607[41] = 0;
   out_1015750461961158607[42] = 0;
   out_1015750461961158607[43] = 0;
   out_1015750461961158607[44] = 0;
   out_1015750461961158607[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1015750461961158607[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1015750461961158607[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1015750461961158607[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1015750461961158607[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1015750461961158607[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1015750461961158607[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1015750461961158607[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1015750461961158607[53] = -9.8000000000000007*dt;
   out_1015750461961158607[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1015750461961158607[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1015750461961158607[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1015750461961158607[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1015750461961158607[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1015750461961158607[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1015750461961158607[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1015750461961158607[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1015750461961158607[62] = 0;
   out_1015750461961158607[63] = 0;
   out_1015750461961158607[64] = 0;
   out_1015750461961158607[65] = 0;
   out_1015750461961158607[66] = 0;
   out_1015750461961158607[67] = 0;
   out_1015750461961158607[68] = 0;
   out_1015750461961158607[69] = 0;
   out_1015750461961158607[70] = 1;
   out_1015750461961158607[71] = 0;
   out_1015750461961158607[72] = 0;
   out_1015750461961158607[73] = 0;
   out_1015750461961158607[74] = 0;
   out_1015750461961158607[75] = 0;
   out_1015750461961158607[76] = 0;
   out_1015750461961158607[77] = 0;
   out_1015750461961158607[78] = 0;
   out_1015750461961158607[79] = 0;
   out_1015750461961158607[80] = 1;
}
void h_25(double *state, double *unused, double *out_579481060641732431) {
   out_579481060641732431[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8838814511200069080) {
   out_8838814511200069080[0] = 0;
   out_8838814511200069080[1] = 0;
   out_8838814511200069080[2] = 0;
   out_8838814511200069080[3] = 0;
   out_8838814511200069080[4] = 0;
   out_8838814511200069080[5] = 0;
   out_8838814511200069080[6] = 1;
   out_8838814511200069080[7] = 0;
   out_8838814511200069080[8] = 0;
}
void h_24(double *state, double *unused, double *out_8628826922044058076) {
   out_8628826922044058076[0] = state[4];
   out_8628826922044058076[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7435279963503982970) {
   out_7435279963503982970[0] = 0;
   out_7435279963503982970[1] = 0;
   out_7435279963503982970[2] = 0;
   out_7435279963503982970[3] = 0;
   out_7435279963503982970[4] = 1;
   out_7435279963503982970[5] = 0;
   out_7435279963503982970[6] = 0;
   out_7435279963503982970[7] = 0;
   out_7435279963503982970[8] = 0;
   out_7435279963503982970[9] = 0;
   out_7435279963503982970[10] = 0;
   out_7435279963503982970[11] = 0;
   out_7435279963503982970[12] = 0;
   out_7435279963503982970[13] = 0;
   out_7435279963503982970[14] = 1;
   out_7435279963503982970[15] = 0;
   out_7435279963503982970[16] = 0;
   out_7435279963503982970[17] = 0;
}
void h_30(double *state, double *unused, double *out_8060396416076809453) {
   out_8060396416076809453[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6320481552692820453) {
   out_6320481552692820453[0] = 0;
   out_6320481552692820453[1] = 0;
   out_6320481552692820453[2] = 0;
   out_6320481552692820453[3] = 0;
   out_6320481552692820453[4] = 1;
   out_6320481552692820453[5] = 0;
   out_6320481552692820453[6] = 0;
   out_6320481552692820453[7] = 0;
   out_6320481552692820453[8] = 0;
}
void h_26(double *state, double *unused, double *out_8663712300753286970) {
   out_8663712300753286970[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5866426243635426312) {
   out_5866426243635426312[0] = 0;
   out_5866426243635426312[1] = 0;
   out_5866426243635426312[2] = 0;
   out_5866426243635426312[3] = 0;
   out_5866426243635426312[4] = 0;
   out_5866426243635426312[5] = 0;
   out_5866426243635426312[6] = 0;
   out_5866426243635426312[7] = 1;
   out_5866426243635426312[8] = 0;
}
void h_27(double *state, double *unused, double *out_2233071042628120952) {
   out_2233071042628120952[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4096887481508877236) {
   out_4096887481508877236[0] = 0;
   out_4096887481508877236[1] = 0;
   out_4096887481508877236[2] = 0;
   out_4096887481508877236[3] = 1;
   out_4096887481508877236[4] = 0;
   out_4096887481508877236[5] = 0;
   out_4096887481508877236[6] = 0;
   out_4096887481508877236[7] = 0;
   out_4096887481508877236[8] = 0;
}
void h_29(double *state, double *unused, double *out_8528880318658063037) {
   out_8528880318658063037[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5810250208378428269) {
   out_5810250208378428269[0] = 0;
   out_5810250208378428269[1] = 1;
   out_5810250208378428269[2] = 0;
   out_5810250208378428269[3] = 0;
   out_5810250208378428269[4] = 0;
   out_5810250208378428269[5] = 0;
   out_5810250208378428269[6] = 0;
   out_5810250208378428269[7] = 0;
   out_5810250208378428269[8] = 0;
}
void h_28(double *state, double *unused, double *out_4837476363436818603) {
   out_4837476363436818603[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7554094848261592773) {
   out_7554094848261592773[0] = 1;
   out_7554094848261592773[1] = 0;
   out_7554094848261592773[2] = 0;
   out_7554094848261592773[3] = 0;
   out_7554094848261592773[4] = 0;
   out_7554094848261592773[5] = 0;
   out_7554094848261592773[6] = 0;
   out_7554094848261592773[7] = 0;
   out_7554094848261592773[8] = 0;
}
void h_31(double *state, double *unused, double *out_7350316286992083367) {
   out_7350316286992083367[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5240218141402074836) {
   out_5240218141402074836[0] = 0;
   out_5240218141402074836[1] = 0;
   out_5240218141402074836[2] = 0;
   out_5240218141402074836[3] = 0;
   out_5240218141402074836[4] = 0;
   out_5240218141402074836[5] = 0;
   out_5240218141402074836[6] = 0;
   out_5240218141402074836[7] = 0;
   out_5240218141402074836[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1840749225442711100) {
  err_fun(nom_x, delta_x, out_1840749225442711100);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7887258947850824197) {
  inv_err_fun(nom_x, true_x, out_7887258947850824197);
}
void car_H_mod_fun(double *state, double *out_5849575757077882696) {
  H_mod_fun(state, out_5849575757077882696);
}
void car_f_fun(double *state, double dt, double *out_1306455665935687231) {
  f_fun(state,  dt, out_1306455665935687231);
}
void car_F_fun(double *state, double dt, double *out_1015750461961158607) {
  F_fun(state,  dt, out_1015750461961158607);
}
void car_h_25(double *state, double *unused, double *out_579481060641732431) {
  h_25(state, unused, out_579481060641732431);
}
void car_H_25(double *state, double *unused, double *out_8838814511200069080) {
  H_25(state, unused, out_8838814511200069080);
}
void car_h_24(double *state, double *unused, double *out_8628826922044058076) {
  h_24(state, unused, out_8628826922044058076);
}
void car_H_24(double *state, double *unused, double *out_7435279963503982970) {
  H_24(state, unused, out_7435279963503982970);
}
void car_h_30(double *state, double *unused, double *out_8060396416076809453) {
  h_30(state, unused, out_8060396416076809453);
}
void car_H_30(double *state, double *unused, double *out_6320481552692820453) {
  H_30(state, unused, out_6320481552692820453);
}
void car_h_26(double *state, double *unused, double *out_8663712300753286970) {
  h_26(state, unused, out_8663712300753286970);
}
void car_H_26(double *state, double *unused, double *out_5866426243635426312) {
  H_26(state, unused, out_5866426243635426312);
}
void car_h_27(double *state, double *unused, double *out_2233071042628120952) {
  h_27(state, unused, out_2233071042628120952);
}
void car_H_27(double *state, double *unused, double *out_4096887481508877236) {
  H_27(state, unused, out_4096887481508877236);
}
void car_h_29(double *state, double *unused, double *out_8528880318658063037) {
  h_29(state, unused, out_8528880318658063037);
}
void car_H_29(double *state, double *unused, double *out_5810250208378428269) {
  H_29(state, unused, out_5810250208378428269);
}
void car_h_28(double *state, double *unused, double *out_4837476363436818603) {
  h_28(state, unused, out_4837476363436818603);
}
void car_H_28(double *state, double *unused, double *out_7554094848261592773) {
  H_28(state, unused, out_7554094848261592773);
}
void car_h_31(double *state, double *unused, double *out_7350316286992083367) {
  h_31(state, unused, out_7350316286992083367);
}
void car_H_31(double *state, double *unused, double *out_5240218141402074836) {
  H_31(state, unused, out_5240218141402074836);
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
