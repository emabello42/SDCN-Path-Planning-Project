#ifndef VEHICLE_H
#define VEHICLE_H

using namespace std;


enum Vstate {KL, PLCL, PLCR, LCL, LCR};
enum WheelCommand {KEEP_DIR, TURN_LEFT, TURN_RIGHT};
enum SpeedCommand {KEEP_SPEED, SPEED_UP, SPEED_DOWN};

struct CarCommand {
    WheelCommand wheel;
    SpeedCommand speed;
};

class Vehicle {
public:
    double x;
    double y;
    double s;
    double d;
    int lane;
    double yaw;
    double v;
    double a;
    double t;
    Vstate state;

public:
    double positionAt(double dt);
    double speedAt(double dt);
};

#endif
