#include "ptg.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


vector<vector<double>> PTG::generate(const vector<Vehicle> & highLevelTrajectory,
                                    const map<int, Vehicle> & predictions,
                                    double dt, int nPoints)
{
    double T = dt*nPoints;
    double t;
    Vehicle startVehicle = highLevelTrajectory[0];
    Vehicle goalVehicle = highLevelTrajectory[1];
    
    Tstate start; 
    start.s = startVehicle.s_;
    start.s_dot = startVehicle.v_;
    start.s_dot_dot = startVehicle.a_;
    start.d = startVehicle.d_;
    start.d_dot = 0;
    start.d_dot_dot = 0;
    
    Tstate goal;
    goal.s = goalVehicle.s_;
    goal.s_dot = goalVehicle.v_;
    goal.s_dot_dot = goalVehicle.a_;
    goal.d = goalVehicle.d_;
    goal.d_dot = 0;
    goal.d_dot_dot = 0;

    t = T - 4*dt;
    vector<vector<double>> goals;
    while(t <= (T+ 4*dt))
    {
        for(int k = 0; k < N_SAMPLES; ++k)
        {
            vector<double> pgoal_s = {goal_s[0], goal_s[1], goal_s[2]};
            vector<double> pgoal_d = {goal_d[0], goal_d[1], goal_d[2]};
            perturbGoal(pgoal_s, pgoal_d);
            
        }
        t += dt;
    }
}

void PTG::perturbGoal(vector<double> & rgoal_s, vector<double> rgoal_d)
{
}

vector<double> PTG::JMT(vector<double> start, vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.
    INPUTS
    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]
    end   - the desired end state for vehicle. Like "start" this is a
        length three array.
    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    */

    double T2 = T*T;
    double T3 = T2*T;
    double T4 = T3*T;
    double T5 = T4*T;
    MatrixXd A(3,3);
    A << T3, T4, T5,
         3*T2, 4*T3, 5*T4,
         6*T, 12*T2, 20*T3;

    double si = start[0];
    double si_p = start[1];
    double si_pp = start[2];
    double sf = end[0];
    double sf_p = end[1];
    double sf_pp = end[2];

    VectorXd b(3);
    b << sf - (si + si_p*T + 0.5*si_pp*T2),
         sf_p - (si_p + si_pp*T),
         sf_pp - si_pp;
    MatrixXd x = A.colPivHouseholderQr().solve(b);

    return {si, si_p, 0.5*si_pp, x(0), x(1), x(2)};
}
