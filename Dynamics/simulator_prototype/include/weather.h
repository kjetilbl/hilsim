#ifndef WEATHER_H
#define WEATHER_H

#include <random>
#include <cmath>
#include <map>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d;

// This module contains models for wind and current. The module outputs a
// current and wind speed and angle at any given time, and it is left up to the
// vessel module to convert this to forces for the given vessel.
// Setting new wind speed should be done by inputting the desired wind speed at
// 10 metres above the surface, U_10
class Weather {
public:
  // Speed and angle for environmental forces. Angle defined as 0=going to
  // north.
    Weather();
    ~Weather();
    void setData(double V_in, double beta_in, double dt);
protected:
    double mean_beta, mean_V, V, beta;
    double mu_V, mu_beta, dt;
    double generateRandomSpeed();
    double generateRandomAngle();
    double maintainSaturation(double current_value, double mean_value, double max_deviation);
    void updateSpeed();
    void updateDirection();
    double max_V_deviation, max_beta_deviation;
};

#endif