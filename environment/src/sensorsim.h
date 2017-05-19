#ifndef SENSORSIM_H
#define SENSORSIM_H

#include <QTimer>
#include <QObject>
#include <QThread>
#include <QDebug>

#include <string>
#include <map>
#include <mutex>

#include "navData.h"
#include "gpsTools.h"

#include "ros/ros.h"
#include "environment/obstacleUpdate.h"
#include "simulator_messages/Gps.h"
#include "simulator_messages/detectedTarget.h"

using namespace std;

class detectedObject;

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
	void AIS_parser(const simulator_messages::AIS::ConstPtr& AISmsg);

	bool is_within_visibility(gpsPoint obstaclePosition, double crossSection);

	navData USVnavData;

	mutex m;
	double radarRange = 100;
	ros::NodeHandle nh;
	ros::Publisher detectedTargetPub;
	QTimer *AIStimer;
	QTimer *DTtimer;
	map<string, detectedObject> unidentifiedObjects;
	map<int, navData> detectedAISusers;
};

class detectedObject
{
public:
	detectedObject(gpsPointStamped truePosition, double trueCrossSection, double trueCOG, double trueSOG);
	detectedObject(){};
	void make_parameter_estimates(double distanceFromUSV);
	uint32_t get_target_number(){ return targetNumber; }
	gpsPointStamped get_estimated_position(){ return estimatedPos; }
	double get_estimated_CS(){ return estimatedCS; }
	double get_estimated_COG(){ return estimatedCOG; }
	double get_estimated_SOG(){ return estimatedSOG; }
	gpsPointStamped truePos;
	double trueCS; //m²
	double trueCOG;
	double trueSOG;
 
private:
	static uint32_t targetIterator;
	uint32_t targetNumber;
	gpsPointStamped estimatedPos;
	double estimatedCS; //m²
	double estimatedCOG;
	double estimatedSOG;
};


#endif // SENSORSIM_H