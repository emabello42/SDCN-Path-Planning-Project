#include "vehicle.h"


double Vehicle::positionAt(double dt)
{
    return s + v*dt + 0.5*a*dt*dt;
}

double Vehicle::speedAt(double dt)
{
    return v + a*dt;
}
