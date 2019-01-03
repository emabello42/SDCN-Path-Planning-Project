#ifndef PTG_H
#define PTG_H
#include "vehicle.h"
#include <vector>
#include <map>
#include "ptg_cost.h"

using namespace std;


class PTG {
public:
    Trajectory generate(const Vehicle & startVehicle, const Vehicle & goalVehicle,
                        const map<int, Vehicle> & predictions, double T);

private:
    vector<double> JMT(vector<double> start, vector<double> end, double T);
    Tstate perturbGoal(Tstate goal);

private:
    PTGCost ptgCost_;
};
#endif
