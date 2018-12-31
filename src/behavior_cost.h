#ifndef BEHAVIOR_COST_H
#define BEHAVIOR_COST_H
#include "vehicle.h"
#include <map>
#include <vector>

using namespace std;

class BehaviorCost {

public:
    BehaviorCost();
    virtual ~BehaviorCost();

    double calculateCost(double targetSpeed, const map<int, Vehicle> & predictions, const vector<Vehicle> & trajectory);

private:

    double inefficiencyCost(double targetSpeed, const vector<Vehicle> & trajectory, const map<int, Vehicle> & predictions, map<string, double> & data);

    double laneSpeed(const map<int, Vehicle> & predictions, int lane);

    map<string, double> getHelperData(const vector<Vehicle> & trajectory, const map<int, Vehicle> & predictions);

private:
    const double EFFICIENCY = 1.0;
};
#endif
