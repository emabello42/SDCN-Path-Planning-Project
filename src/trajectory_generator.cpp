#include "trajectory_generator.h"
#include <math.h>
#include "spline.h"
#include "helpers.h"
#include <iostream>

vector<PathPoint> TrajectoryGenerator::generate(
                             const Vehicle & locData,
                             const vector<vector<double>> & sensorFusion,
                             const vector<double> & previous_path_x,
                             const vector<double> & previous_path_y,
                             const double end_path_s, const double end_path_d)
{
    /*
     * Generate a trajectory of points
     * Arguments:
     *  - locdata: current information about the location of the ego Car
     *  - sensorFusion: data about other cars on the road.
     *  - previos_path_x and previous_path_y: (x,y) coordinates of the points
     *  generated but not visited by the ego Car yet.
     *  - end_path_s and end_path_d: (s,d) coordinates of the ego Car after
     *  visiting the remaining points in the previous path
     */
    vector<PathPoint> pathPoints;
    
    // later we will interpolate these waypoints with a spline and fill
    // it in with more points that control speed
    vector<double> ptsx; 
    vector<double> ptsy;

    int prev_size = previous_path_x.size();

    //measure time
    if(prev_size > 0)
    {
        timestamp_ += (nPoints_-prev_size)*0.02;//update time
    }
    else
    {
        timestamp_ = 0;
    }

    updateLocation(locData);
    behaviorPlanner_.updatePredictions(sensorFusion, timestamp_);
    
    // refCar is the reference from which we generete the trajectory
    // either we will reference the starting point as where the egoCar
    // is or at the previous paths end point
    Vehicle refCar;
    refCar.x = egoCar_.x;
    refCar.y = egoCar_.y;
    refCar.yaw = egoCar_.yaw;
    refCar.s = egoCar_.s;
    refCar.d = egoCar_.d;
    refCar.state = egoCar_.state;
    if(prev_size > 0)
    {
        refCar.s = end_path_s;
    }
    if(prev_size < 2)
    {
        //Use two points that make the path tangent to the car
        double prev_car_x = egoCar_.x - cos(refCar.yaw);
        double prev_car_y = egoCar_.y - sin(refCar.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(egoCar_.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(egoCar_.y);
    }
    else
    {
        // Use the previous path's end point as starting point

        // Redefine reference state as previous path end point
        refCar.x = previous_path_x[prev_size-1];
        refCar.y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        refCar.yaw = atan2(refCar.y - ref_y_prev, refCar.x - ref_x_prev);

        //Use two points that make the path tangent to the previous
        //path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(refCar.x);
                
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(refCar.y);
    }
    //behavior planner decides the next action to take and which is going to be
    //the next state of the egoCar
    Vstate nextState;
    refCar.lane = refLane_;

    CarCommand carCmd = behaviorPlanner_.getNextAction(refCar, speedLimit_, nextState, prev_size*0.02);
    egoCar_.state = nextState;//update state
    executeWheelCommand(carCmd.wheel);
    
    refCar.d = (laneWidth_/2)+laneWidth_*refLane_;
    vector<double> next_wp0 = getXY(refCar.s+30, refCar.d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    vector<double> next_wp1 = getXY(refCar.s+60, refCar.d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    vector<double> next_wp2 = getXY(refCar.s+90, refCar.d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);
            
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);
    for(int i=0; i < ptsx.size(); i++)
    {
        //shift car reference angle to 0 degrees
        double dx = ptsx[i] - refCar.x;
        double dy = ptsy[i] - refCar.y;
        ptsx[i] = dx * cos(0 - refCar.yaw) - dy * sin(0 - refCar.yaw);
        ptsy[i] = dx * sin(0 - refCar.yaw) + dy * cos(0 - refCar.yaw);
    }
    // create a spline
    tk::spline s;
    
    // set(x,y) points to the spline
    s.set_points(ptsx, ptsy);

    // Start with all of the previous path points from last time
    for(int i=0; i < prev_size; i++)
    {
        PathPoint point;
        point.x = previous_path_x[i];
        point.y = previous_path_y[i];
        pathPoints.push_back(point);
    }
            
    // Calculate how to break up the spline points so that we travel at
    // our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with
    // previous points, here we will always output 50 points
    for(int i=1; i <= (nPoints_ - prev_size); i++)
    {
        PathPoint point;
        //execute speed command, checking the speed
        if(carCmd.speed == SPEED_DOWN && referenceSpeed_ > 0)
        {
           referenceSpeed_ -= 0.1; 
        }
        else if(carCmd.speed == SPEED_UP && referenceSpeed_< speedLimit_)
        {
           referenceSpeed_ += 0.1;
        }
        
        if(referenceSpeed_ == 0.0)//car must stop, do not generate more points
            break;

        double N = (target_dist/(.02*referenceSpeed_));
        double x_point = x_add_on+ target_x/N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        //rotate back to normal after rotating it earlier
        x_point = x_ref * cos(refCar.yaw) - y_ref * sin(refCar.yaw);
        y_point = x_ref * sin(refCar.yaw) + y_ref * cos(refCar.yaw);
        x_point += refCar.x;
        y_point += refCar.y;
        
        point.x = x_point;
        point.y = y_point;
        pathPoints.push_back(point);
    }//end fo

    return pathPoints; 
}

void TrajectoryGenerator::updateLocation(const Vehicle & locData)
{
    double prev_v = egoCar_.v;
    double prev_t = egoCar_.t;
    egoCar_.x = locData.x;
    egoCar_.y = locData.y;
    egoCar_.s = locData.s;
    egoCar_.d = locData.d;
    egoCar_.yaw = locData.yaw;
    egoCar_.v = locData.v;
    egoCar_.t = timestamp_;
    if(timestamp_ > 0)
    {
        egoCar_.a = (locData.v - prev_v)/(timestamp_ - prev_t);
    }
    else
    {
        egoCar_.a = 0;
    }
}

void TrajectoryGenerator::executeWheelCommand(WheelCommand cmd)
{
    switch(cmd)
    {
        case TURN_LEFT:
            refLane_ -= 1;
            break;
        case TURN_RIGHT:
            refLane_ += 1;
            break;
        //or just keep the same lane
    }
}
