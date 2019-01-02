#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "vehicle.h"
#include "ptg.h"
#include "behavior_cost.h"

using namespace std;


class BehaviorPlanner {
public:
  /**
  * Constructor
  */
  BehaviorPlanner(double laneWidth, int nLanes)
      : laneWidth_(laneWidth), nLanes_(nLanes)
  {
      car_.v_ = 0.0;
      car_.a_ = 0.0;
      car_.timestamp_ = 0.0;
  }

  /**
  * Destructor
  */
  ~BehaviorPlanner() {};
  void updateLocation(const vector<double> & location, double speedLimit);
  void updatePredictions(const vector<vector<double>> & sensorFusion);
  vector<vector<double>> generateTrajectory(double dt, int nPoints);

private:
  vector<Vstate> successorStates();
  vector<Vehicle> generateHighLevelTrajectory(Vstate state);
  int getLane(double d);
  vector<Vehicle> constanSpeedTrajectory();
  vector<Vehicle> keepLaneTrajectory();
  vector<Vehicle> laneChangeTrajectory(Vstate state);
  vector<Vehicle> prepLaneChangeTrajectory(Vstate state);
  bool getVehicleBehind(int lane, Vehicle & rVehicle);
  bool getVehicleAhead(int lane, Vehicle & rVehicle);
  vector<double> getKinematics(int lane);
private:
  map<Vstate, int> mapLaneDirection_ = {{PLCL, -1}, {LCL, -1}, {LCR, 1}, {PLCR, 1}};
  double laneWidth_;
  int nLanes_;
  double dt_;
  int nPoints_;
  double speedLimit_;
  double targetSpeed_;
  double preferredBuffer_;
  Vehicle car_;//ego vehicle
  map<int, Vehicle> predictions_;
  BehaviorCost behaviorCost_;
  PTG ptg_;
  
};

#endif
