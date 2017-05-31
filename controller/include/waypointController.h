#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H
#include <math.h> 
#include "ros/ros.h"

struct gpsPoint {
	double longitude = 0;
	double latitude = 0;
};

class WaypointController{
public:
double deg2rad(double degrees);
double distance_m(gpsPoint a, gpsPoint b);
double compass_bearing(gpsPoint from, gpsPoint to);
double latitude_degs_pr_meter();
double longitude_degs_pr_meter(double currentLatitude);

};

#endif