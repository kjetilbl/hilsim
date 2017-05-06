#ifndef GPSTOOLS_H
#define GPSTOOLS_H


struct gpsPoint {
	double longitude;
	double latitude;
};

struct gpsPoint3DOF : public gpsPoint {
	gpsPoint3DOF(){};
	gpsPoint3DOF(double Longitude, double Latitude, double Heading){
		this->longitude = Longitude;
		this->latitude = Latitude;
		this->heading = Heading;
	};
	double heading;
};

double deg2rad(double degrees);

double distance(gpsPoint a, gpsPoint b);

double latitude_degs_pr_meter();

double longitude_degs_pr_meter(double currentLatitude);

#endif // GPSTOOLS_H