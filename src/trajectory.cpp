#include "trajectory.h"
#include <math.h>

Trajectory::Trajectory(vector<double> s_coeffs, vector<double> d_coeffs, double timestamp_start, double dt)
{
    s = s_coeffs;
    d = d_coeffs;
    t_start = timestamp_start;
    T = dt;

    s_dot = differentiate(s);
    s_dot_dot = differentiate(s_dot);
    s_jerk = differentiate(s_dot_dot);
    
    d_dot = differentiate(d);
    d_dot_dot = differentiate(d_dot);
    d_jerk = differentiate(d_dot_dot);

}

double Trajectory::sPositionAt(double t) const
{
    return evaluatePolynomialAt(s, t);
}

double Trajectory::dPositionAt(double t) const
{
    return evaluatePolynomialAt(d, t);
}

double Trajectory::sVelocityAt(double t) const
{
    return evaluatePolynomialAt(s_dot, t);
}

double Trajectory::dVelocityAt(double t) const
{
    return evaluatePolynomialAt(d_dot, t);
}

double Trajectory::sAccelerationAt(double t) const
{
    return evaluatePolynomialAt(s_dot_dot, t);
}

double Trajectory::dAccelerationAt(double t) const
{
    return evaluatePolynomialAt(d_dot_dot, t);
}

double Trajectory::sJerkAt(double t) const
{
    return evaluatePolynomialAt(s_jerk, t);
}

double Trajectory::dJerkAt(double t) const
{
    return evaluatePolynomialAt(d_jerk, t);
}

double Trajectory::evaluatePolynomialAt(const vector<double> & coeffs, double t) const
{
    /*
     * Takes the coefficients of a polynomial and evaluate it for the input t
     */
    double total = 0.0;
    for(int i = 0; i < coeffs.size(); ++i)
    {
        total += coeffs[i]*pow(t, i);
    }

    return total;
}

vector<double> Trajectory::differentiate(const vector<double> & coeffs) const
{
    /*
    * Calculates the derivative of a polynomial and returns
    * the corresponding coefficients.
    */
    vector<double> new_cos;
    for(int deg = 1; deg < coeffs.size(); deg++)
    {
        new_cos.push_back(deg*coeffs[deg]);
    }
    return new_cos;
}
