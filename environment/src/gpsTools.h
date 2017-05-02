#ifndef GPSTOOLS_H
#define GPSTOOLS_H


struct gpsPoint {
	double longitude;
	double latitude;
};

double deg2rad(double degrees);

double distance(gpsPoint a, gpsPoint b);

double latitude_degs_pr_meter();

double longitude_degs_pr_meter(double currentLatitude);

#endif // GPSTOOLS_H