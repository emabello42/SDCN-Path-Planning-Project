#include "ptg.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include <random>
#include <limits>
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Trajectory PTG::generate(const Vehicle & startVehicle, const Vehicle & goalVehicle, 
                         const map<int, Vehicle> & predictions, double T)
{
    /*
     * Find the best trajectory according to a set of cost functions
     * Arguments:
    */
    double t;
    Trajectory bestTrajectory;

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
    goal.s_dot_dot =0;// goalVehicle.a_;
    goal.d = goalVehicle.d_;
    goal.d_dot = 0;
    goal.d_dot_dot = 0;

    //generate alternative goals
    double timestep = 0.05;
    t = T - 4*timestep;
    vector<Tstate> goals;
    while(t <= (T + 4*timestep))
    {
        for(int k = 0; k < N_SAMPLES; ++k)
        {
            Tstate perturbedGoal = perturbGoal(goal);
            perturbedGoal.t = t;
            goals.push_back(perturbedGoal);
        }
        t += timestep;
    }

    //find best trajectory
    double min_cost = std::numeric_limits<double>::max();
    Tstate best_goal;
    for(Tstate pgoal : goals)
    {
        vector<double> s_coeffs;
        vector<double> d_coeffs;
        s_coeffs = JMT({start.s, start.s_dot, start.s_dot_dot}, {pgoal.s, pgoal.s_dot, pgoal.s_dot_dot}, pgoal.t);
        d_coeffs = JMT({start.d, start.d_dot, start.d_dot_dot}, {pgoal.d, pgoal.d_dot, pgoal.d_dot_dot}, pgoal.t);
        Trajectory tr = Trajectory(s_coeffs, d_coeffs, startVehicle.t_, pgoal.t);
        tr.cost = ptgCost_.calculateCost(tr, pgoal, T, predictions);
        if(tr.cost < min_cost)
        {
            min_cost = tr.cost;
            bestTrajectory = tr;
            best_goal = pgoal;
        }
    }
    cout << "best goal: s= "<< best_goal.s << ", sdot= "<< best_goal.s_dot;
    cout << ", sddot= " << best_goal.s_dot_dot << ", t="<< best_goal.t<< endl;
    cout << "best goal: d= "<< best_goal.d << ", ddot= "<< best_goal.d_dot;
    cout << ", dddot= " << best_goal.d_dot_dot << endl;
    double cost = ptgCost_.calculateCost(bestTrajectory, best_goal, T, predictions, true);
   //cout << "FINAL COST= "<< cost << endl;
    //TODO return trajectory
    return bestTrajectory;
}

Tstate PTG::perturbGoal(Tstate goal)
{
    /*
     * Returns a "perturbed" version of the goal.
     */
    Tstate new_state;
    //default_random_engine gen;
     // random device class instance, source of 'true' randomness for initializing random seed
    std::random_device rd;

    // Mersenne twister PRNG, initialized with seed from previous random device instance
    std::mt19937 gen(rd());

    normal_distribution<double> dist_s(goal.s, SIGMA_S[0]);
    normal_distribution<double> dist_s_dot(goal.s_dot, SIGMA_S[1]);
    normal_distribution<double> dist_s_dot_dot(goal.s_dot_dot, SIGMA_S[2]);
    
    normal_distribution<double> dist_d(goal.d, SIGMA_D[0]);
    normal_distribution<double> dist_d_dot(goal.d_dot, SIGMA_D[1]);
    normal_distribution<double> dist_d_dot_dot(goal.d_dot_dot, SIGMA_D[2]);
    
    new_state.s = dist_s(gen);
    new_state.s_dot = dist_s_dot(gen);
    //new_state.s_dot_dot = dist_s_dot_dot(gen);
    new_state.s_dot_dot = 0;

    new_state.d = dist_d(gen);
    new_state.d_dot = dist_d_dot(gen);
    //new_state.d_dot_dot = dist_d_dot_dot(gen);

    new_state.d_dot_dot = 0;
    return new_state;
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
