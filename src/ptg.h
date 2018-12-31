#ifndef PTG_H
#define PTG_H
#include "vehicle.h"
#include <vector>
#include <map>
#include "ptg_cost.h"

using namespace std;

struct Tstate {
    double s;
    double s_dot;
    double s_dot_dot;
    
    double d;
    double d_dot;
    double d_dot_dot;

    double t;
};

struct TrajectoryCoeffs {
    vector<double> s_coeffs;
    vector<double> d_coeffs;

    double t;
};

class PTG {
public:
    vector<vector<double>> generate(const vector<Vehicle> & highLevelTrajectory,
                                    const map<int, Vehicle> & predictions,
                                    double dt, int nPoints);

private:
    vector<double> JMT(vector<double> start, vector<double> end, double T);
    Tstate perturbGoal(Tstate goal);

private:
    PTGCost ptgCost_;
};
#endif
