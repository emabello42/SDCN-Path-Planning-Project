#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H
#define DEBUG
#include "vehicle.h"
#include <vector>
#include "behavior_cost.h"
#include <map>

using namespace std;


class BehaviorPlanner {
public:
  
  BehaviorPlanner(double laneWidth, int nLanes) : laneWidth_(laneWidth), nLanes_(nLanes){}
  
  CarCommand getNextAction(const Vehicle & refCar, double targetSpeed, Vstate & rNextState, double dt);
  void updatePredictions(const vector<vector<double>> & sensorFusion, double t);
  int getLane(double d);

private:
  vector<Vstate> successorStates(Vstate state);

private:
  map<Vstate, int> mapLaneDirection_ = {{PLCL, -1}, {LCL, -1}, {LCR, 1}, {PLCR, 1}};  
  double laneWidth_;
  int nLanes_;

  map<int, Vehicle> predictions_;
  BehaviorCost behaviorCost_;
  
};

#endif
