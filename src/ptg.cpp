#include "ptg.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen/Eigen/Dense"
#include <random>
#include <limits>
#include "ptg_helpers.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


vector<vector<double>> PTG::generate(const vector<Vehicle> & highLevelTrajectory,
                                    const map<int, Vehicle> & predictions,
                                    double dt, int nPoints)
{
    /*
     * Find the best trajectory according to a set of cost functions
     * Arguments:
     *  - highLevelTrajectory: is a vector of two Vehicle, which contains the
     *  start and goal state that the trajectory should achieve
     *  - predictions: is the list of Vehicles, that contain the positions of
     *  the others vehicles in the road.
     *  - dt: time between each trajectory point
     *  - nPoints: number of trajectory points
     * */
    double T = dt*nPoints;//T is the time that the trajectory must cover.
    double SIGMA_T = 4*dt;
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

    //generate alternative goals
    t = T - SIGMA_T;
    vector<Tstate> goals;
    while(t <= (T + SIGMA_T))
    {
        for(int k = 0; k < N_SAMPLES; ++k)
        {
            Tstate perturbedGoal = perturbGoal(goal);
            perturbedGoal.t = t;
            goals.push_back(perturbedGoal);
        }
        t += dt;
    }

    //find best trajectory
    vector<TrajectoryCoeffs> trajectories;
    for(Tstate pgoal : goals)
    {
        TrajectoryCoeffs tr;
        tr.s_coeffs = JMT({start.s, start.s_dot, start.s_dot_dot}, {pgoal.s, pgoal.s_dot, pgoal.s_dot_dot}, pgoal.t);
        tr.d_coeffs = JMT({start.d, start.d_dot, start.d_dot_dot}, {pgoal.d, pgoal.d_dot, pgoal.d_dot_dot}, pgoal.t);
        tr.t = pgoal.t;
        trajectories.push_back(tr);
    }
    
    TrajectoryCoeffs bestTrajectory;
    double min_cost = std::numeric_limits<double>::max();
    for(int k = 0; k < trajectories.size(); ++k)
    {
        double cost = ptgCost_.calculateCost(trajectories[k], goal, T, predictions);
        if(cost < min_cost)
        {
            min_cost = cost;
            bestTrajectory = trajectories[k];
        }

    }
    vector<vector<double>> result;
    t = 0;
    for(int i = 0; i < nPoints; i++)
    {
        vector<double> point(2);
        point[0] = polynomial_evaluation(bestTrajectory.s_coeffs, t);
        point[1] = polynomial_evaluation(bestTrajectory.d_coeffs, t);
        result.push_back(point);
        t += dt;
    }
    return result;
}

Tstate PTG::perturbGoal(Tstate goal)
{
    /*
     * Returns a "perturbed" version of the goal.
     */
    Tstate new_state;
    default_random_engine gen;
    normal_distribution<double> dist_s(goal.s, SIGMA_S[0]);
    normal_distribution<double> dist_s_dot(goal.s_dot, SIGMA_S[1]);
    normal_distribution<double> dist_s_dot_dot(goal.s_dot_dot, SIGMA_S[2]);
    
    normal_distribution<double> dist_d(goal.d, SIGMA_D[0]);
    normal_distribution<double> dist_d_dot(goal.d_dot, SIGMA_D[1]);
    normal_distribution<double> dist_d_dot_dot(goal.d_dot_dot, SIGMA_D[2]);
    
    new_state.s = dist_s(gen);
    new_state.s_dot = dist_s_dot(gen);
    new_state.s_dot_dot = dist_s_dot_dot(gen);
    
    new_state.d = dist_d(gen);
    new_state.d_dot = dist_d_dot(gen);
    new_state.d_dot_dot = dist_d_dot_dot(gen);

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
