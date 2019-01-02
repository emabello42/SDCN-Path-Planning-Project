#include "ptg_helpers.h"

double logistic(double x)
{
    /*
    * A function that returns a value between 0 and 1 for x in the
    * range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
    * Useful for cost functions.
    */
    
    return 2.0 / (1 + exp(-x)) - 1.0;
}

double polynomial_evaluation(const vector<double> & coeffs, double t)
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

vector<double> differentiate(const vector<double> & coeffs)
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
/*
def nearest_approach_to_any_vehicle(traj, vehicles):
    """
    Calculates the closest distance to any vehicle during a trajectory.
    """
    closest = 999999
    for v in vehicles.values():
        d = nearest_approach(traj,v)
        if d < closest:
            closest = d
    return closest


double nearest_approach(traj, vehicle)
{
    closest = 999999
    s_,d_,T = traj
    s = to_equation(s_)
    d = to_equation(d_)
    for i in range(100):
        t = float(i) / 100 * T
        cur_s = s(t)
        cur_d = d(t)
        targ_s, _, _, targ_d, _, _ = vehicle.state_in(t)
        dist = sqrt((cur_s-targ_s)**2 + (cur_d-targ_d)**2)
        if dist < closest:
            closest = dist
    return closest
}
*/
vector<double> get_f_and_N_derivatives(const vector<double> & coeffs, int derivN, double t)
{
    vector<double> results = {polynomial_evaluation(coeffs, t)};
    vector<double> new_coeffs;
    for(int i=0; i < derivN; ++i)
    {
        new_coeffs = differentiate(coeffs);
        results.push_back(polynomial_evaluation(new_coeffs, t));
    }
    return results;
}
