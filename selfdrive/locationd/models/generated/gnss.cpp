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
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2351947283659598660) {
   out_2351947283659598660[0] = delta_x[0] + nom_x[0];
   out_2351947283659598660[1] = delta_x[1] + nom_x[1];
   out_2351947283659598660[2] = delta_x[2] + nom_x[2];
   out_2351947283659598660[3] = delta_x[3] + nom_x[3];
   out_2351947283659598660[4] = delta_x[4] + nom_x[4];
   out_2351947283659598660[5] = delta_x[5] + nom_x[5];
   out_2351947283659598660[6] = delta_x[6] + nom_x[6];
   out_2351947283659598660[7] = delta_x[7] + nom_x[7];
   out_2351947283659598660[8] = delta_x[8] + nom_x[8];
   out_2351947283659598660[9] = delta_x[9] + nom_x[9];
   out_2351947283659598660[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8539310053726918653) {
   out_8539310053726918653[0] = -nom_x[0] + true_x[0];
   out_8539310053726918653[1] = -nom_x[1] + true_x[1];
   out_8539310053726918653[2] = -nom_x[2] + true_x[2];
   out_8539310053726918653[3] = -nom_x[3] + true_x[3];
   out_8539310053726918653[4] = -nom_x[4] + true_x[4];
   out_8539310053726918653[5] = -nom_x[5] + true_x[5];
   out_8539310053726918653[6] = -nom_x[6] + true_x[6];
   out_8539310053726918653[7] = -nom_x[7] + true_x[7];
   out_8539310053726918653[8] = -nom_x[8] + true_x[8];
   out_8539310053726918653[9] = -nom_x[9] + true_x[9];
   out_8539310053726918653[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_2843304307356815987) {
   out_2843304307356815987[0] = 1.0;
   out_2843304307356815987[1] = 0;
   out_2843304307356815987[2] = 0;
   out_2843304307356815987[3] = 0;
   out_2843304307356815987[4] = 0;
   out_2843304307356815987[5] = 0;
   out_2843304307356815987[6] = 0;
   out_2843304307356815987[7] = 0;
   out_2843304307356815987[8] = 0;
   out_2843304307356815987[9] = 0;
   out_2843304307356815987[10] = 0;
   out_2843304307356815987[11] = 0;
   out_2843304307356815987[12] = 1.0;
   out_2843304307356815987[13] = 0;
   out_2843304307356815987[14] = 0;
   out_2843304307356815987[15] = 0;
   out_2843304307356815987[16] = 0;
   out_2843304307356815987[17] = 0;
   out_2843304307356815987[18] = 0;
   out_2843304307356815987[19] = 0;
   out_2843304307356815987[20] = 0;
   out_2843304307356815987[21] = 0;
   out_2843304307356815987[22] = 0;
   out_2843304307356815987[23] = 0;
   out_2843304307356815987[24] = 1.0;
   out_2843304307356815987[25] = 0;
   out_2843304307356815987[26] = 0;
   out_2843304307356815987[27] = 0;
   out_2843304307356815987[28] = 0;
   out_2843304307356815987[29] = 0;
   out_2843304307356815987[30] = 0;
   out_2843304307356815987[31] = 0;
   out_2843304307356815987[32] = 0;
   out_2843304307356815987[33] = 0;
   out_2843304307356815987[34] = 0;
   out_2843304307356815987[35] = 0;
   out_2843304307356815987[36] = 1.0;
   out_2843304307356815987[37] = 0;
   out_2843304307356815987[38] = 0;
   out_2843304307356815987[39] = 0;
   out_2843304307356815987[40] = 0;
   out_2843304307356815987[41] = 0;
   out_2843304307356815987[42] = 0;
   out_2843304307356815987[43] = 0;
   out_2843304307356815987[44] = 0;
   out_2843304307356815987[45] = 0;
   out_2843304307356815987[46] = 0;
   out_2843304307356815987[47] = 0;
   out_2843304307356815987[48] = 1.0;
   out_2843304307356815987[49] = 0;
   out_2843304307356815987[50] = 0;
   out_2843304307356815987[51] = 0;
   out_2843304307356815987[52] = 0;
   out_2843304307356815987[53] = 0;
   out_2843304307356815987[54] = 0;
   out_2843304307356815987[55] = 0;
   out_2843304307356815987[56] = 0;
   out_2843304307356815987[57] = 0;
   out_2843304307356815987[58] = 0;
   out_2843304307356815987[59] = 0;
   out_2843304307356815987[60] = 1.0;
   out_2843304307356815987[61] = 0;
   out_2843304307356815987[62] = 0;
   out_2843304307356815987[63] = 0;
   out_2843304307356815987[64] = 0;
   out_2843304307356815987[65] = 0;
   out_2843304307356815987[66] = 0;
   out_2843304307356815987[67] = 0;
   out_2843304307356815987[68] = 0;
   out_2843304307356815987[69] = 0;
   out_2843304307356815987[70] = 0;
   out_2843304307356815987[71] = 0;
   out_2843304307356815987[72] = 1.0;
   out_2843304307356815987[73] = 0;
   out_2843304307356815987[74] = 0;
   out_2843304307356815987[75] = 0;
   out_2843304307356815987[76] = 0;
   out_2843304307356815987[77] = 0;
   out_2843304307356815987[78] = 0;
   out_2843304307356815987[79] = 0;
   out_2843304307356815987[80] = 0;
   out_2843304307356815987[81] = 0;
   out_2843304307356815987[82] = 0;
   out_2843304307356815987[83] = 0;
   out_2843304307356815987[84] = 1.0;
   out_2843304307356815987[85] = 0;
   out_2843304307356815987[86] = 0;
   out_2843304307356815987[87] = 0;
   out_2843304307356815987[88] = 0;
   out_2843304307356815987[89] = 0;
   out_2843304307356815987[90] = 0;
   out_2843304307356815987[91] = 0;
   out_2843304307356815987[92] = 0;
   out_2843304307356815987[93] = 0;
   out_2843304307356815987[94] = 0;
   out_2843304307356815987[95] = 0;
   out_2843304307356815987[96] = 1.0;
   out_2843304307356815987[97] = 0;
   out_2843304307356815987[98] = 0;
   out_2843304307356815987[99] = 0;
   out_2843304307356815987[100] = 0;
   out_2843304307356815987[101] = 0;
   out_2843304307356815987[102] = 0;
   out_2843304307356815987[103] = 0;
   out_2843304307356815987[104] = 0;
   out_2843304307356815987[105] = 0;
   out_2843304307356815987[106] = 0;
   out_2843304307356815987[107] = 0;
   out_2843304307356815987[108] = 1.0;
   out_2843304307356815987[109] = 0;
   out_2843304307356815987[110] = 0;
   out_2843304307356815987[111] = 0;
   out_2843304307356815987[112] = 0;
   out_2843304307356815987[113] = 0;
   out_2843304307356815987[114] = 0;
   out_2843304307356815987[115] = 0;
   out_2843304307356815987[116] = 0;
   out_2843304307356815987[117] = 0;
   out_2843304307356815987[118] = 0;
   out_2843304307356815987[119] = 0;
   out_2843304307356815987[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_774769938980785647) {
   out_774769938980785647[0] = dt*state[3] + state[0];
   out_774769938980785647[1] = dt*state[4] + state[1];
   out_774769938980785647[2] = dt*state[5] + state[2];
   out_774769938980785647[3] = state[3];
   out_774769938980785647[4] = state[4];
   out_774769938980785647[5] = state[5];
   out_774769938980785647[6] = dt*state[7] + state[6];
   out_774769938980785647[7] = dt*state[8] + state[7];
   out_774769938980785647[8] = state[8];
   out_774769938980785647[9] = state[9];
   out_774769938980785647[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4467768114936024642) {
   out_4467768114936024642[0] = 1;
   out_4467768114936024642[1] = 0;
   out_4467768114936024642[2] = 0;
   out_4467768114936024642[3] = dt;
   out_4467768114936024642[4] = 0;
   out_4467768114936024642[5] = 0;
   out_4467768114936024642[6] = 0;
   out_4467768114936024642[7] = 0;
   out_4467768114936024642[8] = 0;
   out_4467768114936024642[9] = 0;
   out_4467768114936024642[10] = 0;
   out_4467768114936024642[11] = 0;
   out_4467768114936024642[12] = 1;
   out_4467768114936024642[13] = 0;
   out_4467768114936024642[14] = 0;
   out_4467768114936024642[15] = dt;
   out_4467768114936024642[16] = 0;
   out_4467768114936024642[17] = 0;
   out_4467768114936024642[18] = 0;
   out_4467768114936024642[19] = 0;
   out_4467768114936024642[20] = 0;
   out_4467768114936024642[21] = 0;
   out_4467768114936024642[22] = 0;
   out_4467768114936024642[23] = 0;
   out_4467768114936024642[24] = 1;
   out_4467768114936024642[25] = 0;
   out_4467768114936024642[26] = 0;
   out_4467768114936024642[27] = dt;
   out_4467768114936024642[28] = 0;
   out_4467768114936024642[29] = 0;
   out_4467768114936024642[30] = 0;
   out_4467768114936024642[31] = 0;
   out_4467768114936024642[32] = 0;
   out_4467768114936024642[33] = 0;
   out_4467768114936024642[34] = 0;
   out_4467768114936024642[35] = 0;
   out_4467768114936024642[36] = 1;
   out_4467768114936024642[37] = 0;
   out_4467768114936024642[38] = 0;
   out_4467768114936024642[39] = 0;
   out_4467768114936024642[40] = 0;
   out_4467768114936024642[41] = 0;
   out_4467768114936024642[42] = 0;
   out_4467768114936024642[43] = 0;
   out_4467768114936024642[44] = 0;
   out_4467768114936024642[45] = 0;
   out_4467768114936024642[46] = 0;
   out_4467768114936024642[47] = 0;
   out_4467768114936024642[48] = 1;
   out_4467768114936024642[49] = 0;
   out_4467768114936024642[50] = 0;
   out_4467768114936024642[51] = 0;
   out_4467768114936024642[52] = 0;
   out_4467768114936024642[53] = 0;
   out_4467768114936024642[54] = 0;
   out_4467768114936024642[55] = 0;
   out_4467768114936024642[56] = 0;
   out_4467768114936024642[57] = 0;
   out_4467768114936024642[58] = 0;
   out_4467768114936024642[59] = 0;
   out_4467768114936024642[60] = 1;
   out_4467768114936024642[61] = 0;
   out_4467768114936024642[62] = 0;
   out_4467768114936024642[63] = 0;
   out_4467768114936024642[64] = 0;
   out_4467768114936024642[65] = 0;
   out_4467768114936024642[66] = 0;
   out_4467768114936024642[67] = 0;
   out_4467768114936024642[68] = 0;
   out_4467768114936024642[69] = 0;
   out_4467768114936024642[70] = 0;
   out_4467768114936024642[71] = 0;
   out_4467768114936024642[72] = 1;
   out_4467768114936024642[73] = dt;
   out_4467768114936024642[74] = 0;
   out_4467768114936024642[75] = 0;
   out_4467768114936024642[76] = 0;
   out_4467768114936024642[77] = 0;
   out_4467768114936024642[78] = 0;
   out_4467768114936024642[79] = 0;
   out_4467768114936024642[80] = 0;
   out_4467768114936024642[81] = 0;
   out_4467768114936024642[82] = 0;
   out_4467768114936024642[83] = 0;
   out_4467768114936024642[84] = 1;
   out_4467768114936024642[85] = dt;
   out_4467768114936024642[86] = 0;
   out_4467768114936024642[87] = 0;
   out_4467768114936024642[88] = 0;
   out_4467768114936024642[89] = 0;
   out_4467768114936024642[90] = 0;
   out_4467768114936024642[91] = 0;
   out_4467768114936024642[92] = 0;
   out_4467768114936024642[93] = 0;
   out_4467768114936024642[94] = 0;
   out_4467768114936024642[95] = 0;
   out_4467768114936024642[96] = 1;
   out_4467768114936024642[97] = 0;
   out_4467768114936024642[98] = 0;
   out_4467768114936024642[99] = 0;
   out_4467768114936024642[100] = 0;
   out_4467768114936024642[101] = 0;
   out_4467768114936024642[102] = 0;
   out_4467768114936024642[103] = 0;
   out_4467768114936024642[104] = 0;
   out_4467768114936024642[105] = 0;
   out_4467768114936024642[106] = 0;
   out_4467768114936024642[107] = 0;
   out_4467768114936024642[108] = 1;
   out_4467768114936024642[109] = 0;
   out_4467768114936024642[110] = 0;
   out_4467768114936024642[111] = 0;
   out_4467768114936024642[112] = 0;
   out_4467768114936024642[113] = 0;
   out_4467768114936024642[114] = 0;
   out_4467768114936024642[115] = 0;
   out_4467768114936024642[116] = 0;
   out_4467768114936024642[117] = 0;
   out_4467768114936024642[118] = 0;
   out_4467768114936024642[119] = 0;
   out_4467768114936024642[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8206867657584711155) {
   out_8206867657584711155[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2748205046550560630) {
   out_2748205046550560630[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2748205046550560630[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2748205046550560630[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2748205046550560630[3] = 0;
   out_2748205046550560630[4] = 0;
   out_2748205046550560630[5] = 0;
   out_2748205046550560630[6] = 1;
   out_2748205046550560630[7] = 0;
   out_2748205046550560630[8] = 0;
   out_2748205046550560630[9] = 0;
   out_2748205046550560630[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3144072620339266513) {
   out_3144072620339266513[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_8303647801952025828) {
   out_8303647801952025828[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8303647801952025828[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8303647801952025828[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8303647801952025828[3] = 0;
   out_8303647801952025828[4] = 0;
   out_8303647801952025828[5] = 0;
   out_8303647801952025828[6] = 1;
   out_8303647801952025828[7] = 0;
   out_8303647801952025828[8] = 0;
   out_8303647801952025828[9] = 1;
   out_8303647801952025828[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_7730590468597126339) {
   out_7730590468597126339[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_1168235388167943403) {
   out_1168235388167943403[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[6] = 0;
   out_1168235388167943403[7] = 1;
   out_1168235388167943403[8] = 0;
   out_1168235388167943403[9] = 0;
   out_1168235388167943403[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_7730590468597126339) {
   out_7730590468597126339[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_1168235388167943403) {
   out_1168235388167943403[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1168235388167943403[6] = 0;
   out_1168235388167943403[7] = 1;
   out_1168235388167943403[8] = 0;
   out_1168235388167943403[9] = 0;
   out_1168235388167943403[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2351947283659598660) {
  err_fun(nom_x, delta_x, out_2351947283659598660);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8539310053726918653) {
  inv_err_fun(nom_x, true_x, out_8539310053726918653);
}
void gnss_H_mod_fun(double *state, double *out_2843304307356815987) {
  H_mod_fun(state, out_2843304307356815987);
}
void gnss_f_fun(double *state, double dt, double *out_774769938980785647) {
  f_fun(state,  dt, out_774769938980785647);
}
void gnss_F_fun(double *state, double dt, double *out_4467768114936024642) {
  F_fun(state,  dt, out_4467768114936024642);
}
void gnss_h_6(double *state, double *sat_pos, double *out_8206867657584711155) {
  h_6(state, sat_pos, out_8206867657584711155);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2748205046550560630) {
  H_6(state, sat_pos, out_2748205046550560630);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3144072620339266513) {
  h_20(state, sat_pos, out_3144072620339266513);
}
void gnss_H_20(double *state, double *sat_pos, double *out_8303647801952025828) {
  H_20(state, sat_pos, out_8303647801952025828);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7730590468597126339) {
  h_7(state, sat_pos_vel, out_7730590468597126339);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1168235388167943403) {
  H_7(state, sat_pos_vel, out_1168235388167943403);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7730590468597126339) {
  h_21(state, sat_pos_vel, out_7730590468597126339);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1168235388167943403) {
  H_21(state, sat_pos_vel, out_1168235388167943403);
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
