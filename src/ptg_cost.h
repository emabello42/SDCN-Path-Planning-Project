#ifndef PTG_COST_H
#define PTG_COST_H
#include "vehicle.h"
#include <vector>
#include "ptg_constants.h"

using namespace std;

struct Tstate;
struct TrajectoryCoeffs;
//double SIGMA_T;//calculated in runtime based on dt

class PTGCost {
public:
    double calculateCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);

private:
    double timeDiffCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double sDiffCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double dDiffCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double collisionCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double bufferCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double efficiencyCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double totalAccelCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double maxAccelCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double maxJerkCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);
    double totalJerkCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions);

private:
    const double TIME_DIFF_COST = 1.0;
    const double S_DIFF_COST = 1.0;
    const double D_DIFF_COST = 1.0;
    const double EFFICIENCY_COST = 0.0;
    const double MAX_JERK_COST = 0.0;
    const double TOTAL_JERK_COST = 0.0;
    const double COLLISION_COST = 0.0;
    const double BUFFER_COST = 0.0;
    const double MAX_ACCEL_COST = 0.0;
    const double TOTAL_ACCEL_COST = 0.0;
};
#endif
