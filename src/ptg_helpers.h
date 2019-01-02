#ifndef PTG_HELPERS_H
#define PTG_HELPERS_H
#include <math.h>
#include <vector>
using namespace std;

double logistic(double x);
double polynomial_evaluation(const vector<double> & coeffs, double t);
vector<double> differentiate(const vector<double> & coeffs);
vector<double> get_f_and_N_derivatives(const vector<double> & coeffs, int derivN, double t);

#endif
