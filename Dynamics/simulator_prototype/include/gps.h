#ifndef GPS_H
#define GPS_H

#include "sensor.h"
#include "solver.h"

typedef Matrix<double, 6, 6> Matrix6d;

class GPS : public Sensor {
public:
  GPS();
  ~GPS();
  // Receives the start position for the vessel.
  void setCoordinates(double latitute_start, double longitude_start);

  void publishGpsData(Vector6d nu_n, Vector6d eta);

  void getCoordinates(double &lat_, double &long_);

private:
  // Latitude and longitude used for GPS position.
  long double latitude, longitude, height;
  // Prime and meridian curvatures of earth
  long double r_mer, r_prime;
  // WGS-84 parameters needed for transformation
  long double r_e = 6378137;
  long double e = 0.0818191908426215;
  double heading;


  NumericalSolver solver;

  Matrix6d A;

  double getHeading(Vector6d eta);

  void updateCurvatures();

  void calculateNextPosition();

  double getSpeed();
  double getTrack();
  double getHeadingRate();

  Vector6d positionFunction(Vector6d position_in);

  Vector6d gps_position, v_n;


  // Uses the NED velocity received to calculate the change in GPS coordinates,
  // and integrates to get the new GPS coordinates.
};

#endif