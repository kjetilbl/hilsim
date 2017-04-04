#ifndef GPS_H
#define GPS_H


class gpsData
{
public:
	double longitude;
	double latitude;
	double heading;
	double headingRate;
	double speed;
	double altitude;
	gpsData(double Longitude, double Latitude, double Heading = 0, double HeadingRate = 0, double Speed = 0, double Altitude = 0);
	gpsData();

private:
};


#endif // GPS_H

