#ifndef VEHICLE_H
#define VEHICLE_H

using namespace std;

enum Vstate {CS, KL, LCL, LCR, PLCL, PLCR};

class Vehicle {
public:
  /**
  * Constructor
  */
  Vehicle(int lane=0, double s=0.0, double d=0.0, double v=0.0, double a=0.0,
          Vstate state=CS) : state_(state), lane_(lane), s_(s), d_(d), v_(v), a_(a)
    {
    }

  /**
  * Destructor
  */
  virtual ~Vehicle();
  double s_;
  double d_;
  int lane_;
  double v_;
  double a_;
  double timestamp_;
  Vstate state_;  
  double positionAt(double dt);
};

#endif
