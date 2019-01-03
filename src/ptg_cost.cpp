#include "ptg_cost.h"
#include <math.h>
#include <iostream>

inline double logistic(double x)
{
    /*
    * A function that returns a value between 0 and 1 for x in the
    * range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
    * Useful for cost functions.
    */
    
    return 2.0 / (1 + exp(-x)) - 1.0;
}

double PTGCost::calculateCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions, bool verbose)
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

double PTGCost::timeDiffCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return logistic(fabs(trajectory.T-T)/T);
}

double PTGCost::sDiffCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    /*
    * Penalizes trajectories whose s coordinate (and derivatives)
    * differ from the goal.
    */
    double cost = 0.0;
    cost += logistic(fabs(trajectory.sPositionAt(trajectory.T) - goal.s)/SIGMA_S[0]);
    cost += logistic(fabs(trajectory.sVelocityAt(trajectory.T) - goal.s_dot)/SIGMA_S[1]);
    cost += logistic(fabs(trajectory.sAccelerationAt(trajectory.T) - goal.s_dot_dot)/SIGMA_S[2]);

    return cost;
}

double PTGCost::dDiffCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    /*
    * Penalizes trajectories whose d coordinate (and derivatives)
    * differ from the goal.
    */
    double cost = 0.0;
    cost += logistic(fabs(trajectory.dPositionAt(trajectory.T) - goal.d)/SIGMA_D[0]);
    cost += logistic(fabs(trajectory.dVelocityAt(trajectory.T) - goal.d_dot)/SIGMA_D[1]);
    cost += logistic(fabs(trajectory.dAccelerationAt(trajectory.T) - goal.d_dot_dot)/SIGMA_D[2]);

    return cost;
}

double PTGCost::collisionCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return 0;
}

double PTGCost::bufferCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    return 0;
}

double PTGCost::efficiencyCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    if(trajectory.T == 0)
        return 1;
    double avg_v =  trajectory.sPositionAt(trajectory.T)/ trajectory.T;
    double targ_v = goal.s/trajectory.T;
    if(targ_v == 0)
        return 1;
    return logistic(2*(targ_v-avg_v)/avg_v);
}

double PTGCost::totalAccelCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    double dt = T/100.0;
    double total_acc = 0;
    double t, acc;
    for(int i=0; i < 100; i++)
    {
        t = dt*i;
        acc = trajectory.sAccelerationAt(t);
        total_acc += fabs(acc*dt);
    }
    double acc_per_second = total_acc /T;
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC);
}

double PTGCost::maxAccelCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    double max_acc = 0;
    double acc;
    for(int i= 0; i < 100 ; i++)
    {
        acc = trajectory.sAccelerationAt((T/100)*i);
        if(fabs(acc) > max_acc)
        {
            max_acc = fabs(acc);
        }
    }
    if(max_acc > MAX_ACCEL) return 1;
    else return 0;
}

double PTGCost::maxJerkCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    double jerk;
    double max_jerk = 0;
    for(int i=0; i < 100; i++)
    {
        jerk = trajectory.sJerkAt((T/100.0)*i);
        if(fabs(jerk) > max_jerk)
        {
            max_jerk = fabs(jerk);
        }
    }
    if(max_jerk > MAX_JERK) return 1;
    else return 0;
}

double PTGCost::totalJerkCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    double dt = T/100.0;
    double total_jerk = 0;
    double jerk, t;
    for(int i=0; i< 100; i++)
    {
        t = dt*i;
        jerk = trajectory.sJerkAt(t);
        total_jerk += fabs(jerk*dt);
    }
    double jerk_per_second = total_jerk / T;
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC );
}

double PTGCost::exceedsSpeedLimitCost(const Trajectory & trajectory, Tstate goal, double T, const map<int, Vehicle> & predictions)
{
    double v;
    double max_v = 0;
    for(int i=0; i < 100; i++)
    {
        v = trajectory.sVelocityAt((T/100.0)*i);
        if(fabs(v) > max_v)
        {
            max_v = fabs(v);
        }
    }
    if(max_v > goal.s_dot) return 1;
    else return 0;
}
