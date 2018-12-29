#ifndef VEHICLE_H
#define VEHICLE_H

using namespace std;

class Vehicle {
public:
  /**
  * Constructor
  */
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();
  double id;
  double s;
  double d;
  double speed;
  double acc;
  double timestamp;

};

#endif
