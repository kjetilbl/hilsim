#include "gpsTools.h"
#include <math.h>
#include <cmath>
#include <QDebug>

double deg2rad(double degrees){
	return degrees*M_PI/180;
}

double distance_m(gpsPoint a, gpsPoint b)
{
	// https://en.wikipedia.org/wiki/Haversine_formula
	// Cannot guarantee accuracy better than 0.5%
	
	double deg2rad = M_PI/180;
	double h = pow(sin((b.latitude - a.latitude)*deg2rad/2), 2) + cos(a.latitude*deg2rad)*cos(b.latitude*deg2rad)*pow(sin((b.longitude - a.longitude)*deg2rad/2), 2);
	if ( h >= 1 )
		qDebug() << "Warning: in function distance_m(), h >= 1, which might give floating point error.";
	double r = 6378000; // meters, assumed constant
	double d = 2*r*asin(sqrt(h));
	return abs(d);
}

double compass_bearing(gpsPoint from, gpsPoint to){
	double straightDist = distance_m(from, to);
	double northDist = (from.latitude - to.latitude)/latitude_degs_pr_meter();
	double eastDist = (from.longitude - to.longitude)/longitude_degs_pr_meter(from.latitude);
	double y = sin( deg2rad(to.longitude-from.longitude) ) * cos( deg2rad(to.latitude) );
	double x = cos(deg2rad(from.latitude))*sin(deg2rad(to.latitude))
			 - sin(deg2rad(from.latitude))*cos(deg2rad(to.latitude))*cos(deg2rad(to.longitude - from.longitude));
	double bearing = atan2(y,x)*180/M_PI;
	if(bearing < 0) bearing += 360;
	if(bearing >= 360) bearing -= 360;

	return bearing;
}

bool is_within_bearing_range(double bearing, double min, double max){
	int intMin = floor(min);
	int intMax = ceil(max);
	int intBearing = round(bearing);
	intMin = intMin % 360;
	int degsFromMinToNorth = 360 - intMin;
	intMax = (intMax + degsFromMinToNorth) % 360;
	intBearing = (intBearing + degsFromMinToNorth) % 360;
	if (intBearing <= intMax)
	{
		return true;
	}
	return false;
}

double latitude_degs_pr_meter()
{
	double r = 6378000; // meters, assumed constant
	return 90/(2*M_PI*r/4);
}

double longitude_degs_pr_meter(double currentLatitude)
{
	double deg2rad = M_PI/180;
	double r = 6378000; // meters, assumed constant
	return 180/(2*M_PI*r*cos(currentLatitude*deg2rad)/2);
}