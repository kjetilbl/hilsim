#ifndef SENSORSIM_H
#define SENSORSIM_H

#include <QTimer>
#include <QObject>
#include <QThread>
#include <QDebug>
#include <string>
#include <map>

#include "gps.h"

#include "ros/ros.h"
#include "environment/obstacleUpdate.h"
#include "simulator_messages/Gps.h"

using namespace std;

class sensorSim : public QThread
{
	Q_OBJECT 
public:
	sensorSim( ros::NodeHandle n, QThread *parent = 0 );
	~sensorSim();

private slots:
	void publish_AIS();
	void publish_detected_target_msg();

private:
	void run();
	void gps_parser(const simulator_messages::Gps::ConstPtr& USVgpsMsg);
	void position_update_parser(const environment::obstacleUpdate::ConstPtr& obstUpdateMsg);

	ros::NodeHandle nh;
	QTimer *AIStimer;
	QTimer *DTtimer;

	gpsData USVpos;
	map<string, gpsData> obstPositions; // TODO gpsData inneholder time, bruk denne til å disregarde gamle updates.
};


#endif // SENSORSIM_H

