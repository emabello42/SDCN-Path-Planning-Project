#ifndef VEHICLE_H
#define VEHICLE_H

#include "vehicle.h"
#include "trajectory_generator.h"

enum Vstate {CS, KL, LCL, LCR, PLCL, PLCR};
using namespace std;

class BehaviorPlanner {
public:
  /**
  * Constructor
  */
  BehaviorPlanner(double dt, int nPoints) : state_(CS), dt_(dt), nPoints_(nPoints)
  {
      egoVehicle_.speed = 0.0;
      egoVehicle_.acc = 0.0;
      egoVehicle_.timestamp = 0.0;
  }

  /**
  * Destructor
  */
  virtual ~BehaviorPlanner();
  void updateLocation(const vector<double> & location);
  void updatePredictions(const vector<vector<double>> & sensorFusion, double timestamp);
  vector<vector<double>> generateTrajectory();

private:
  vector<VState> successorStates();
  vector<Vehicle> generateHighLevelTrajectory(const Vstata state);
  double calculateCost(const vector<Vehicle> & trajectory);

  vector<Vehicle> constanSpeedTrajectory();
  vector<Vehicle> keepLaneTrajectory();
  vector<Vehicle> laneChangeTrajectory(Vstate state);
  vector<Vehicle> prepLaneChangeTrajectory(Vstate state);

private:
  double dt_; //period of time between trajectory points
  int nPoints_; //number of points that should contain the trajectory
  Vstate state_;
  Vehicle egoVehicle_;
  map<int, Vehicle> predictions_;
  TrajectoryGenerator trajectoryGenerator_;
  
};

#endif
