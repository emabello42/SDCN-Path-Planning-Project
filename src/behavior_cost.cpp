#include "behavior_cost.h"

double BehaviorCost::calculateCost(double targetSpeed, const map<int, Vehicle> & predictions, const vector<Vehicle> & trajectory)
{
    /*
     * Sum weighted cost functions to get total cost for trajectory.
     */
    map<string, float> trajectory_data = getHelperData(trajectory, predictions);
    float cost = 0.0;

    //Add additional cost functions here.
    vector< function<double(double, const vector<Vehicle> &, const map<int, Vehicle> &, map<string, float> &)>> cf_list = {inefficiencyCost};
    vector<double> weight_list = {EFFICIENCY};

    for (int i = 0; i < cf_list.size(); i++) {
        double new_cost = weight_list[i]*cf_list[i](targetSpeed, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }

    return cost;
}

double BehaviorCost::inefficiencyCost(double targetSpeed, const vector<Vehicle> & trajectory, const map<int, Vehicle> & predictions, map<string, double> & data)
{
    /*
     * Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
     */
    double cost = 1.0;
    double proposed_speed_intended = laneSpeed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = targetSpeed;
    }

    double proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = targetSpeed;
    }

    if(targetSpeed > 0.000001)
    {
        cost = (2.0*targetSpeed - proposed_speed_intended - proposed_speed_final)/targetSpeed;
    }
    return cost;
}

double BehaviorCost::laneSpeed(const map<int, Vehicle> & predictions, int lane)
{
    /*
     * All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
     * we can just find one vehicle in that lane.
     */
    for (map<int, Vehicle>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second;
        if (vehicle.lane_ == lane && key != -1) {
            return vehicle.v_;
        }
    }
    //Found no vehicle in the lane
    return -1.0;
}

map<string, double> BehaviorCost::getHelperData(const vector<Vehicle> & trajectory, const map<int, Vehicle> & predictions)
{
    /*
     * Generate helper data to use in cost functions:
     * indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
     * final_lane: the lane of the vehicle at the end of the trajectory.
     * distance_to_goal: the distance of the vehicle to the goal.
     *
     * Note that indended_lane and final_lane are both included to help differentiate between planning and executing
     * a lane change in the cost functions.
     */
    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[trajectory.size()-1];
    float intended_lane;

    if (trajectory_last.state_ == PLCL) {
        intended_lane = trajectory_last.lane_ - 1;
    } else if (trajectory_last.state_ == PLCR) {
        intended_lane = trajectory_last.lane_ + 1;
    } else {
        intended_lane = trajectory_last.lane_;
    }

    float final_lane = trajectory_last.lane_;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    return trajectory_data;
}
