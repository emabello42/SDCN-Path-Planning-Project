#ifndef PTG_COST_H
#define PTG_COST_H
#include "vehicle.h"
#include <vector>
#include <map>
#include "ptg_constants.h"
#include "trajectory.h"

using namespace std;

class PTGCost {
public:
    double calculateCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions, bool verbose=false);

private:
    double timeDiffCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double sDiffCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double dDiffCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double collisionCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double bufferCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double efficiencyCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double totalAccelCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double maxAccelCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double maxJerkCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double totalJerkCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double exceedsSpeedLimitCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);

private:
    const double TIME_DIFF_COST = 1.0;
    const double S_DIFF_COST = 1.0;
    const double D_DIFF_COST = 1.0;
    const double EFFICIENCY_COST = 1.0;
    const double MAX_JERK_COST = 100.0;
    const double TOTAL_JERK_COST = 10.0;
    const double COLLISION_COST = 1.0;
    const double BUFFER_COST = 1.0;
    const double MAX_ACCEL_COST = 100.0;
    const double TOTAL_ACCEL_COST = 10.0;
    const double EXCEEDS_SPEED_LIMIT_COST = 100.0;
};
#endif
