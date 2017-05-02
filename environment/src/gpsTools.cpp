#include "gpsTools.h"
#include <math.h>  
#include <QDebug>

double deg2rad(double degrees){
	return degrees*M_PI/180;
}

double distance(gpsPoint a, gpsPoint b)
{
	// https://en.wikipedia.org/wiki/Haversine_formula
	// Cannot guarantee accuracy better than 0.5%
	
	double deg2rad = M_PI/180;
	double h = pow(sin((b.latitude - a.latitude)*deg2rad/2), 2) + cos(a.latitude*deg2rad)*cos(b.latitude*deg2rad)*pow(sin((b.longitude - a.longitude)*deg2rad/2), 2);
	if ( h >= 1 )
		qDebug() << "Warning: in function distance(), h >= 1, which might give floating point error.";
	double r = 6378000; // meters, assumed constant
	double d = 2*r*asin(sqrt(h));
	return abs(d);
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