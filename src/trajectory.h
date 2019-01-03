#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <vector>
using namespace std;

struct Tstate {
    double s;
    double s_dot;
    double s_dot_dot;
    
    double d;
    double d_dot;
    double d_dot_dot;

    double t;
};
class Trajectory {
public:
    Trajectory() {};
    Trajectory(vector<double> s_coeffs, vector<double> d_coeffs, double timestamp_start, double dt);

    double sPositionAt(double t) const;
    double dPositionAt(double t) const;

    double sVelocityAt(double t) const;
    double dVelocityAt(double t) const;
    
    double sAccelerationAt(double t) const;
    double dAccelerationAt(double t) const;

    double sJerkAt(double t) const;
    double dJerkAt(double t) const;

private:
    double evaluatePolynomialAt(const vector<double> & coeffs, double t) const;
    vector<double> differentiate(const vector<double> & coeffs) const;

private:
    //S coefficients
    vector<double> s;
    vector<double> s_dot;
    vector<double> s_dot_dot;
    vector<double> s_jerk;

    //D coefficients
    vector<double> d;
    vector<double> d_dot;
    vector<double> d_dot_dot;
    vector<double> d_jerk;

public:
    double t_start;
    double T;
    double cost;
};
#endif
