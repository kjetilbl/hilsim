#ifndef NAVDATA_H
#define NAVDATA_H

#include <QTime>

#include <string>
#include "stdint.h"
#include <math.h>

#include "gpsTools.h"

#include "simulator_messages/AIS.h"

using namespace std;

enum navStatus
{
	UNDERWAY_USING_ENGINE = 0,
	AT_ANCHOR = 1,
	NOT_UNDER_COMMAND = 2,
	RESTRICTED_MANEUVERABILITY = 3,
	CONSTRAINED_BY_DRAUGHT = 4,
	MOORED = 5,
	AGROUND = 6,
	ENGAGED_IN_FISHING = 7,
	UNDER_WAY_SAILING = 8,
	AIS_SART = 14,
	UNDEFINED = 15
};

enum posAccuracy
{
	LOW = 0,
	HIGH = 1
};

enum specialManeuverIndicator
{
	NOT_AVAILABLE = 0,
	NOT_ENGANGED_IN_SPECIAL_MANEUVER = 1,
	ENGAGED_IN_SPECIAL_MANEUVER = 2
};



class navData
{
public:
	navData(uint32_t userID = 0, 
			double longitd = 0, 
			double latitd = 0,
			double SOGmeterPrSec = 0,
			double trackDeg = 0);

	string get_AIS_class_A_position_report() const;
	simulator_messages::AIS get_AIS_ros_msg();
	void print_data() const;
	
	void set_nav_status(navStatus ns);
	navStatus get_nav_status() {return status;};
	void set_ROT(double degPerSec );
	double get_ROT() {return ROT;};
	void set_SOG(double meterPrSec);
	double get_SOG() {return SOG;}
	void set_position_accuracy(posAccuracy pa);
	posAccuracy get_position_accuracy() {return positionAccuracy;}
	void set_position(double longitd, double latitd);
	double get_longitude() {return longitude;}
	double get_latitude() {return latitude;}
	gpsPointStamped get_position() { return gpsPointStamped(longitude, latitude, track, time); }
	void set_COG(double degrees);
	double get_COG() {return COG;}
	void set_track(double degrees);
	double get_track(){return track;}
	void set_time(QTime t);
	
private:
	uint32_t MMSI;
	navStatus status;
	double ROT;
	double SOG;
	posAccuracy positionAccuracy;
	double longitude;
	double latitude;
	double COG;
	double track;
	QTime time;
	specialManeuverIndicator SMI;
};

#endif // NAVDATA_H

