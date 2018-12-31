#ifndef PTG_CONSTANTS_H
#define PTG_CONSTANTS_H
#include <vector>
using namespace std;
static const int N_SAMPLES = 10;
static const vector<double> SIGMA_S = {10.0, 4.0, 2.0}; // s, s_dot, s_dot_dot
static const vector<double> SIGMA_D = {1.0, 1.0, 1.0}; // d, d_dot, d_dot_dot

#endif
