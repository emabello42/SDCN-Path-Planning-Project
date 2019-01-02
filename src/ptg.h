#ifndef PTG_H
#define PTG_H
#include "vehicle.h"
#include <vector>
#include <map>
#include "ptg_cost.h"

using namespace std;


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
