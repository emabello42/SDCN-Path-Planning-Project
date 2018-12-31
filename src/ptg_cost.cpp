#include "ptg_cost.h"
#include <math.h>
#include "ptg_helpers.h"

double PTGCost::calculateCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    //Add additional cost functions here
    vector< function<double(const TrajectoryCoeffs &, Tstate, double, const map<int, Vehicle> &)>> cf_list = 
        {timeDiffCost, sDiffCost, dDiffCost, collisionCost, bufferCost, efficiencyCost, totalAccelCost, maxAccelCost, maxJerkCost, totalJerkCost};
    vector<double> weight_list =
        {TIME_DIFF_COST, S_DIFF_COST, D_DIFF_COST, COLLISION_COST, BUFFER_COST, EFFICIENCY_COST, TOTAL_ACCEL_COST, MAX_ACCEL_COST, MAX_JERK_COST, TOTAL_JERK_COST};
    double cost = 0.0;
    for (int i = 0; i < cf_list.size(); i++)
    {
        double new_cost = weight_list[i]*cf_list[i](trajectory, goal, T, predictions);
        cost += new_cost;
    }
    return cost;
}

double PTGCost::timeDiffCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return logistic(fabs(trajectory.t-T)/T);
}

double PTGCost::sDiffCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    /*
    * Penalizes trajectories whose s coordinate (and derivatives)
    * differ from the goal.
    */
    vector<double> S = get_f_and_N_derivatives(trajectory.s_coeffs, 2, trajectory.t);
    double cost = 0.0;
    cost += logistic(fabs(S[0]-goal.s)/SIGMA_S[0]);
    cost += logistic(fabs(S[1]-goal.s_dot)/SIGMA_S[1]);
    cost += logistic(fabs(S[2]-goal.s_dot_dot)/SIGMA_S[2]);

    return cost;
}

double PTGCost::dDiffCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    /*
    * Penalizes trajectories whose d coordinate (and derivatives)
    * differ from the goal.
    */
    vector<double> D = get_f_and_N_derivatives(trajectory.d_coeffs, 2, trajectory.t);
    double cost = 0.0;
    cost += logistic(fabs(D[0]-goal.d)/SIGMA_D[0]);
    cost += logistic(fabs(D[1]-goal.d_dot)/SIGMA_D[1]);
    cost += logistic(fabs(D[2]-goal.d_dot_dot)/SIGMA_D[2]);

    return cost;
}

double PTGCost::collisionCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return 0;
}

double PTGCost::bufferCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return 0;
}

double PTGCost::efficiencyCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return 0;
}

double PTGCost::totalAccelCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return 0;
}

double PTGCost::maxAccelCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return 0;
}

double PTGCost::maxJerkCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return 0;
}

double PTGCost::totalJerkCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return 0;
}
