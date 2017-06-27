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

	bool is_within_visibility(gpsPoint obstaclePosition, double crossSection, string objectID);

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
	Eigen::VectorXd estimate_states( double distanceFromUSV_m );
	uint32_t get_target_number() { return targetNumber; } // TODO: make const
	gpsPoint get_true_position() { return gpsPoint(X(0), X(1)); }
	double get_true_CS(){ return X(4); }
	gpsPoint get_estimated_position(){ return gpsPoint(Xe(0), Xe(1)); }
	double get_estimated_COG(){ return Xe(2); }
	double get_estimated_SOG(){ return Xe(3); }
	double get_estimated_CS(){ return Xe(4); }
	void update_true_states(gpsPoint pos, double COG, double SOG, double crossSection);
	int msecs_since_last_update(){ return lastUpdate.msecsTo(QTime::currentTime()); }
	simulator_messages::detectedTarget makeDTmsg();
	void set_true_position(gpsPoint truePos);
	void set_true_COG(double trueCOG);
	void set_true_SOG(double trueSOG);
	void set_true_cross_section(double trueCS);
	void set_radar_data(gpsPoint pos, double crossSection);
	void set_AIS_data(double SOG, double COG, double ROT, gpsPoint position);
	bool noiseEnabled = true;
	bool AIS_ON = false;
	bool radar_ON = false;
	string descriptor;
 
private:
	bool read_sensor_config(ros::NodeHandle nh, gpsPoint truePosition);
	Eigen::VectorXd generate_radar_measurement(double distanceFromUSV_m );
	Eigen::VectorXd KalmanFusion();

	bool firstTimeParamEstimate = true;
	QTime lastEstimateTime;
	QTime lastAISupdate;
	QTime lastRADARupdate;
	static uint32_t targetIterator;
	uint32_t targetNumber;
	QTime lastUpdate;
	Eigen::VectorXd X; // true states
	Eigen::VectorXd Xe; // estimated states

	Eigen::VectorXd Zr;			// radar measurements
	Eigen::VectorXd br; 		// radar bias
	Eigen::MatrixXd Tr; 		// continous-time radar bias system matrix
	Eigen::MatrixXd Tdr; 		// discrete-time bias system matrix
	Eigen::VectorXd brSigmas; 	// characteristic standard deviations of radar bias white noise
	Eigen::VectorXd ZrSigmas; // characteristic standard deviations of measurement white noise
	Eigen::MatrixXd errorPrDistanceGain;

	// Kalman parameters
	int n = 5; 			// number of states to track
	int nr = 3; 		// number of states measured by radar/lidar
	int na = 5;		 	// number of states received from AIS
	int nk = 12; 		// number of states in Kalman filter
	double hr = 0.1;	// update interval of radar
	double ha = 2;		// update interval of AIS
	Eigen::VectorXd Za; 		// AIS measurements
	Eigen::VectorXd Za_prev; 	// Previous AIS measurements
	Eigen::VectorXd Xr_bar;
	Eigen::VectorXd Xr_hat;
	Eigen::VectorXd Xa_bar;
	Eigen::VectorXd Xa_hat;
	Eigen::MatrixXd Pa;
	Eigen::MatrixXd Pr;
	Eigen::MatrixXd Pa_bar;
	Eigen::MatrixXd Pr_bar;
	Eigen::MatrixXd fa_hat;
};


#endif // SENSORSIM_H