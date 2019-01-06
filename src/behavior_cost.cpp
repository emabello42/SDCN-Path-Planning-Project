#include "behavior_cost.h"
#include "math.h"
double BehaviorCost::calculateCost(double targetSpeed, const Vehicle & refCar, Lane & finalLane, Lane & intendedLane)
{
    /*
     * Sum weighted cost functions to get total cost for trajectory.
     */
    double cost = 0.0;
    cost += EFFICIENCY * inefficiencyCost(targetSpeed, finalLane, intendedLane);
    cost += TRAFFIC_DENSITY * trafficDensityCost(finalLane);
    return cost;
}

double BehaviorCost::inefficiencyCost(double targetSpeed, Lane & finalLane, Lane & intendedLane)
{
    /*
     * Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
     */
    double cost = 1.0;
    double proposed_speed_intended;
    if (intendedLane.noTraffic)
        proposed_speed_intended = targetSpeed;
    else
        proposed_speed_intended = intendedLane.trafficSpeed;

    double proposed_speed_final;
    if (finalLane.noTraffic)
        proposed_speed_final = targetSpeed;
    else
        proposed_speed_final = finalLane.trafficSpeed;

    if(targetSpeed > 0)
    {
        cost = (2.0*targetSpeed - proposed_speed_intended - proposed_speed_final)/targetSpeed;
    }
    return cost;
}

double BehaviorCost::trafficDensityCost(Lane & finalLane)
{
    return 2.0 / (1 + exp(-finalLane.trafficDensityAhead)) - 1.0;
}

