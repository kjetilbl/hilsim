#ifndef SENSORSIM_H
#define SENSORSIM_H

#include <QTimer>
#include <QObject>
#include <QThread>
#include <QDebug>
#include <string>
#include <map>
#include <mutex>

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
	void print_USV_AIS_msg();
	void print_detected_targets();

private:
	void run();
	void USV_gps_parser(const simulator_messages::Gps::ConstPtr& USVgpsMsg);
	void obstacle_update_parser(const environment::obstacleUpdate::ConstPtr& obstUpdateMsg);

	mutex m;
	ros::NodeHandle nh;
	QTimer *AIStimer;
	QTimer *DTtimer;
	map<string, gpsData> obstPositions;

	navData USVnavData;
};


#endif // SENSORSIM_H

