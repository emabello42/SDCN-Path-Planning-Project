#ifndef PTG_CONSTANTS_H
#define PTG_CONSTANTS_H
#include <vector>
using namespace std;
static const int N_SAMPLES = 10;
static const vector<double> SIGMA_S = {50.0, 4.0, 2.0}; // s, s_dot, s_dot_dot
static const vector<double> SIGMA_D = {1.0, 1.0, 1.0}; // d, d_dot, d_dot_dot
static const double MAX_JERK = 10.0; // m/s/s/s
static const double MAX_ACCEL = 10.0; // m/s/s
static const double EXPECTED_JERK_IN_ONE_SEC = 2.0; // m/s/s
static const double EXPECTED_ACC_IN_ONE_SEC = 1.0; // m/s
static const double VEHICLE_RADIUS = 1.5; // model vehicle as circle to simplify collision detection
#endif
