#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H
#include <vector>
#include "behavior_planner.h"

using namespace std;


struct PathPoint {
    double x;
    double y;
};


class TrajectoryGenerator {
public:
    
    TrajectoryGenerator(int nPoints, double speedLimit, double laneWidth, int nLanes, 
              vector<double> & map_waypoints_s, vector<double> &  map_waypoints_x,
              vector<double>& map_waypoints_y) :
        nPoints_(nPoints), speedLimit_(speedLimit), laneWidth_(laneWidth), nLanes_(nLanes),
        timestamp_(0), referenceSpeed_(0), behaviorPlanner_(laneWidth, nLanes),
        map_waypoints_s_(map_waypoints_s), map_waypoints_x_(map_waypoints_x), map_waypoints_y_(map_waypoints_y)
    {
        refLane_ = 1;
    }
    
    vector<PathPoint> generate(const Vehicle & locData,
                               const vector<vector<double>> & sensorFusion,
                               const vector<double> & previous_path_x,
                               const vector<double> & previous_path_y,
                               const double end_path_s, const double end_path_d);
private:
    void updateLocation(const Vehicle & locData);
    void executeWheelCommand(WheelCommand cmd);

private:
    double speedLimit_;
    double timestamp_;//measure the time
    double referenceSpeed_;//current speed
    Vehicle egoCar_;//keeps information about ego car's locatization

    int nPoints_;
    double laneWidth_;
    int nLanes_;
    int refLane_;//current lane

    //map
    vector<double> & map_waypoints_x_;
    vector<double> & map_waypoints_y_;
    vector<double> & map_waypoints_s_;
    
    BehaviorPlanner behaviorPlanner_;
};
#endif
