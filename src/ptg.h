#ifndef PTG_H
#define PTG_H
#include "vehicle.h"

using namespace std;

struct Tstate {
    double s;
    double s_dot;
    double s_dot_dot;
    
    double d;
    double d_dot;
    double d_dot_dot;
};
class PTG {
public:
    vector<vector<double>> generate(const vector<Vehicle> & highLevelTrajectory,
                                    const map<int, Vehicle> & predictions,
                                    double dt, int nPoints);

private:
    vector<double> JMT(vector<double> start, vector<double> end, double T);
    void perturbGoal(vector<double> & rgoal_s, vector<double> rgoal_d);
private:
    const int N_SAMPLES = 10;
    const vector<double> SIGMA_S = {10.0, 4.0, 2.0}; // s, s_dot, s_dot_dot
    const vector<double> SIGMA_D = {1.0, 1.0, 1.0}; // d, d_dot, d_dot_dot
    double SIGMA_T;//calculated in runtime based on dt
};
#endif
