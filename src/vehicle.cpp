#include "vehicle.h"


double Vehicle::positionAt(double dt)
{
    return s_ + v_*dt + 0.5*a_*dt*dt;
}
