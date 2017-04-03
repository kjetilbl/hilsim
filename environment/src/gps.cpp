#include "gps.h"

gpsData::gpsData(double Longitude, double Latitude, double Heading, double HeadingRate, double Speed, double Altitude)
{
	longitude = Longitude;
	latitude = Latitude;
	heading = Heading;
	headingRate = HeadingRate;
	speed = Speed;
	altitude = Altitude;
}

gpsData::gpsData()
{
	longitude = 0;
	latitude = 0;
	heading = 0;
	headingRate = 0;
	speed = 0;
	altitude = 0;
}