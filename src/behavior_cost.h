#ifndef BEHAVIOR_COST_H
#define BEHAVIOR_COST_H
#include "vehicle.h"
#include <vector>
#include "lane.h"

using namespace std;

class BehaviorCost {

public:
    double calculateCost(double targetSpeed, const Vehicle & refCar, Lane & finalLane, Lane & intendedLane);

private:
    double inefficiencyCost(double targetSpeed, Lane & finalLane, Lane & intendedLane);
    double trafficDensityCost(Lane & finalLane);

private:
    const double EFFICIENCY = 50;
    const double TRAFFIC_DENSITY = 50;
};
#endif
