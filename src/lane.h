#ifndef LANE_H
#define LANE_H

#include "vehicle.h"
#include <limits>

using namespace std;

class Lane {
    /* Keeps useful information to evaluate every lane and take the best action*/
public:
    Lane()
    {
        foundVehicleAhead = false;
        distAhead = std::numeric_limits<double>::max();
        foundVehicleBehind = false;
        distBehind = std::numeric_limits<double>::max();
        trafficSpeed = 0;
        trafficDensityAhead = 0;
        samePostionInS = false;
        noTraffic = true;
    }
public:

    Vehicle vehicleAhead;
    bool foundVehicleAhead;
    double distAhead;//distance between ego Car and vehicle ahead

    Vehicle vehicleBehind;
    double distBehind;//distance between ego Car and vehicle behind
    bool foundVehicleBehind;

    bool samePostionInS;//true if there is another vehicle in the same S postion of the ego Car
    //measurements used in cost functions
    double trafficSpeed;
    double trafficDensityAhead;
    bool noTraffic;
};

#endif
