#ifndef SENSORSIM_H
#define SENSORSIM_H

#include <QTimer>
#include <QObject>
#include <QThread>
#include <QDebug>

#include <string>
#include <map>
#include <mutex>
#include <iostream>
#include "Eigen/Dense"

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
	sensorSim( ros::NodeHandle *n, QThread *parent = 0 );
	~sensorSim();

private slots:
	void print_USV_AIS_msg();
	void publish_detected_targets();

private:
	void run();
	bool read_sensor_config();
	void USV_gps_parser(const simulator_messages::Gps::ConstPtr& USVgpsMsg);
	void obstacle_update_parser(const environment::obstacleUpdate::ConstPtr& obstUpdateMsg);
	void AIS_parser(const simulator_messages::AIS::ConstPtr& AISmsg);

	bool is_within_visibility(gpsPoint obstaclePosition, double crossSection);

	navData USVnavData;

	mutex m;
	double radarRange;
	ros::NodeHandle *nh;
	ros::Publisher detectedTargetPub;
	QTimer *AIStimer;
	QTimer *DTtimer;
	map<string, detectedObject> unidentifiedObjects;
	map<int, navData> detectedAISusers;
};

class detectedObject
{
public:
	detectedObject(	ros::NodeHandle nh,
					string objectDescriptor,
					gpsPoint truePosition,
					double trueCOG, 
					double trueSOG, 
					double trueCrossSection);
	detectedObject();
	void make_parameter_estimates(double distanceFromUSV_m );
	uint32_t get_target_number() { return targetNumber; } // TODO: make const
	gpsPoint get_true_position() { return gpsPoint(X(0), X(1)); }
	double get_true_CS(){ return X(4); }
	gpsPoint get_estimated_position(){ return gpsPoint(Xm(0), Xm(1)); }
	double get_estimated_COG(){ return Xm(2); }
	double get_estimated_SOG(){ return Xm(3); }
	double get_estimated_CS(){ return Xm(4); }
	void update_true_states(gpsPoint pos, double COG, double SOG, double crossSection);
	int msecs_since_last_update(){ return lastUpdate.msecsTo(QTime::currentTime()); }
	simulator_messages::detectedTarget makeDTmsg();
	void set_true_position(gpsPoint truePos);
	void set_true_COG(double trueCOG);
	void set_true_SOG(double trueSOG);
	void set_true_cross_section(double trueCS);
	bool noiseEnabled = true;
	string descriptor;
 
private:
	bool read_sensor_config(ros::NodeHandle nh, gpsPoint truePosition);

	bool firstTimeParamEstimate = true;
	QTime lastEstimateTime;
	static uint32_t targetIterator;
	uint32_t targetNumber;
	QTime lastUpdate;
	Eigen::VectorXd X; // true states
	Eigen::VectorXd Xm; // measured states
	Eigen::VectorXd b; // bias
	Eigen::MatrixXd T; // continous-time bias system matrix
	Eigen::MatrixXd Td; // discrete-time bias system matrix
	Eigen::VectorXd biasSigmas; // characteristic standard deviations of bias white noise
	Eigen::VectorXd measureSigmas; // characteristic standard deviations of measurement white noise
	Eigen::MatrixXd errorPrDistanceGain;
};


#endif // SENSORSIM_H