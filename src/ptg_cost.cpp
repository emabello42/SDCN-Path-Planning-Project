#include "ptg_cost.h"
#include <math.h>
#include <iostream>
double PTGCost::calculateCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions, bool verbose)
{
    //Add additional cost functions here
    double cost = 0.0;
    double pcost = 0;
    pcost = timeDiffCost(trajectory, goal, T, predictions);
    cost += TIME_DIFF_COST * pcost;if(verbose) cout << "time diff cost =" << pcost << endl;

    pcost = sDiffCost(trajectory, goal, T, predictions);
    cost += S_DIFF_COST * pcost;if(verbose) cout << "S diff cost =" << pcost << endl;

    pcost = dDiffCost(trajectory, goal, T, predictions);
    cost += D_DIFF_COST * pcost;if(verbose) cout << "D diff cost =" << pcost << endl;

    pcost = collisionCost(trajectory, goal, T, predictions);
    cost += COLLISION_COST * pcost;if(verbose) cout << "collision cost =" << pcost << endl;

    pcost = bufferCost(trajectory, goal, T, predictions);
    cost += BUFFER_COST * pcost;if(verbose) cout << "buffer cost =" << pcost << endl;

    pcost = efficiencyCost(trajectory, goal, T, predictions);
    cost += EFFICIENCY_COST * pcost;if(verbose) cout << "efficiency cost =" << pcost << endl;

    pcost = totalAccelCost(trajectory, goal, T, predictions);
    cost += TOTAL_ACCEL_COST * pcost;if(verbose) cout << "total accel cost =" << pcost << endl;

    pcost = maxAccelCost(trajectory, goal, T, predictions);
    cost += MAX_ACCEL_COST * pcost;if(verbose) cout << "max accel cost =" << pcost << endl;

    pcost = maxJerkCost(trajectory, goal, T, predictions);
    cost += MAX_JERK_COST * pcost;if(verbose) cout << "max jerk cost cost =" << pcost << endl;

    pcost = totalJerkCost(trajectory, goal, T, predictions);
    cost += TOTAL_JERK_COST * pcost;if(verbose) cout << "total jerk cost =" << pcost << endl;
    
    pcost = exceedsSpeedLimitCost(trajectory, goal, T, predictions);
    cost +=  EXCEEDS_SPEED_LIMIT_COST * pcost;if(verbose) cout << "exceeds speed limit cost =" << pcost << endl;
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
    vector<double> s_dot = differentiate(trajectory.s_coeffs);
    vector<double> s_dot_dot = differentiate(s_dot);
    double dt = T/100.0;
    double total_acc = 0;
    double t, acc;
    for(int i=0; i < 100; i++)
    {
        t = dt*i;
        acc = polynomial_evaluation(s_dot_dot, t);
        total_acc += fabs(acc*dt);
    }
    double acc_per_second = total_acc /T;
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC);
}

double PTGCost::maxAccelCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    vector<double> s_dot = differentiate(trajectory.s_coeffs);
    vector<double> s_dot_dot = differentiate(s_dot);
    double max_acc = 0;
    double acc;
    for(int i= 0; i < 100 ; i++)
    {
        acc = polynomial_evaluation(s_dot_dot, (trajectory.t/100)*i);
        if(fabs(acc) > max_acc)
        {
            max_acc = fabs(acc);
        }
    }
    if(max_acc > MAX_ACCEL) return 1;
    else return 0;
}

double PTGCost::maxJerkCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    vector<double> s_dot = differentiate(trajectory.s_coeffs);
    vector<double> s_dot_dot = differentiate(s_dot);
    vector<double> s_jerk = differentiate(s_dot_dot);
    double jerk;
    double max_jerk = 0;
    for(int i=0; i < 100; i++)
    {
        jerk = polynomial_evaluation(s_jerk, (T/100.0)*i);
        if(fabs(jerk) > max_jerk)
        {
            max_jerk = fabs(jerk);
        }
    }
    if(max_jerk > MAX_JERK) return 1;
    else return 0;
}

double PTGCost::totalJerkCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    vector<double> s_dot = differentiate(trajectory.s_coeffs);
    vector<double> s_dot_dot = differentiate(s_dot);
    vector<double> s_jerk = differentiate(s_dot_dot);
    double dt = T/100.0;
    double total_jerk = 0;
    double jerk, t;
    for(int i=0; i< 100; i++)
    {
        t = dt*i;
        jerk = polynomial_evaluation(s_jerk, t);
        total_jerk += fabs(jerk*dt);
    }
    double jerk_per_second = total_jerk / T;
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC );
}

double PTGCost::exceedsSpeedLimitCost(const TrajectoryCoeffs & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    vector<double> s_dot = differentiate(trajectory.s_coeffs);
    double v;
    double max_v = 0;
    for(int i=0; i < 100; i++)
    {
        v = polynomial_evaluation(s_dot, (T/100.0)*i);
        if(fabs(v) > max_v)
        {
            max_v = fabs(v);
        }
    }
    if(max_v > goal.s_dot) return 1;
    else return 0;
}
