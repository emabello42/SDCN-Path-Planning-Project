#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H
#define DEBUG
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
      car_.t_ = 0.0;
  }

  /**
  * Destructor
  */
  ~BehaviorPlanner() {};
  void updateLocation(const vector<double> & location, double speedLimit, double t);
  void updatePredictions(const vector<vector<double>> & sensorFusion, double t);
  Trajectory generateTrajectory(double T);

private:
  vector<Vstate> successorStates();
  Vehicle generateGoal(Vstate state, bool & rerror);
  int getLane(double d);
  Vehicle constanSpeedTrajectory();
  Vehicle keepLaneTrajectory();
  Vehicle laneChangeTrajectory(Vstate state, bool & rerror);
  Vehicle prepLaneChangeTrajectory(Vstate state);
  bool getVehicleBehind(int lane, Vehicle & rVehicle);
  bool getVehicleAhead(int lane, Vehicle & rVehicle);
  vector<double> getKinematics(int lane);
private:
  map<Vstate, int> mapLaneDirection_ = {{PLCL, -1}, {LCL, -1}, {LCR, 1}, {PLCR, 1}};
  double laneWidth_;
  int nLanes_;
  double T_;
  double speedLimit_;
  double targetSpeed_;
  double preferredBuffer_;
  Vehicle car_;//ego vehicle
  map<int, Vehicle> predictions_;
  BehaviorCost behaviorCost_;
  PTG ptg_;
  
};

#endif
