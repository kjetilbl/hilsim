#ifndef GPSTOOLS_H
#define GPSTOOLS_H


struct gpsPoint {
	double longitude = 0;
	double latitude = 0;
};

struct gpsPoint3DOF : public gpsPoint {
	gpsPoint3DOF(){heading = 0;};
	gpsPoint3DOF(double Longitude, double Latitude, double Heading){
		this->longitude = Longitude;
		this->latitude = Latitude;
		this->heading = Heading;
	};
	double heading;
};

double deg2rad(double degrees);

double distance_m(gpsPoint a, gpsPoint b);

double compass_bearing(gpsPoint from, gpsPoint to);

double latitude_degs_pr_meter();

double longitude_degs_pr_meter(double currentLatitude);

#endif // GPSTOOLS_H