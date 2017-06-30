#ifndef targetDetectionModule_H
#define targetDetectionModule_H

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
#include "simulator_messages/obstacleUpdate.h"
#include "simulator_messages/Gps.h"
#include "simulator_messages/detectedTarget.h"

using namespace std;

class detectedObject;

class targetDetectionModule : public QThread
{
	Q_OBJECT 
public:
	targetDetectionModule( ros::NodeHandle *n, QThread *parent = 0 );
	~targetDetectionModule();

private slots:
	void publish_detected_targets();

private:
	void run();
	bool read_sensor_config();
	void USV_gps_parser(const simulator_messages::Gps::ConstPtr& USVgpsMsg);
	void obstacle_update_parser(const simulator_messages::obstacleUpdate::ConstPtr& obstUpdateMsg);
	void AIS_parser(const simulator_messages::AIS::ConstPtr& AISmsg);

	bool is_within_range(gpsPoint pos, double range);
	bool is_visible(gpsPoint obstaclePosition, string objectID);

	navData USVnavData;

	mutex m;
	double radarRange;
	double lidarRange;
	ros::NodeHandle *nh;
	ros::Publisher detectedTargetPub;
	QTimer *DTtimer;
	map<string, detectedObject> detectedTargets;
};

class detectedObject
{
public:
	detectedObject(	ros::NodeHandle nh,
					string objectDescriptor,
					gpsPoint truePosition,
					double trueCOG, 
					double trueSOG, 
					double trueRadius);
	detectedObject();
	Eigen::VectorXd estimate_states( double distanceFromUSV_m );
	uint32_t get_target_number() { return targetNumber; } 
	gpsPoint get_true_position() { return gpsPoint(X(0), X(1)); }
	double get_true_CS(){ return X(4); }
	gpsPoint get_estimated_position(){ return gpsPoint(Xe(0), Xe(1)); }
	double get_estimated_COG(){ return Xe(2); }
	double get_estimated_SOG(){ return Xe(3); }
	double get_estimated_radius(){ return Xe(4); }
	void update_true_states(gpsPoint pos, double COG, double SOG, double radius);
	simulator_messages::detectedTarget make_DT_msg();
	void set_true_position(gpsPoint truePos);
	void set_true_COG(double trueCOG);
	void set_true_SOG(double trueSOG);
	void set_true_radius(double trueCS);
	void set_AIS_data(double SOG, double COG, double ROT, gpsPoint position);
	void set_lidar_active(bool active);
	void set_radar_active(bool active);
	bool is_active();
	string descriptor;
 
private:
	bool read_sensor_config(ros::NodeHandle nh, gpsPoint truePosition);
	Eigen::VectorXd generate_radar_measurement(double distanceFromUSV_m );
	Eigen::VectorXd generate_lidar_measurement( double distanceFromUSV_m );
	Eigen::VectorXd KalmanFusion();

	bool AIS_ON = false;
	bool radar_ON = false;
	bool lidar_ON = false;

	QTime lastRadarEstimateTime;
	QTime lastlidarEstimateTime;
	QTime lastAISupdate;
	QTime lastRadarUpdate;
	QTime lastLidarUpdate;
	static uint32_t targetIterator;
	uint32_t targetNumber;

	Eigen::VectorXd X; // true states
	Eigen::VectorXd Xe; // estimated states

	Eigen::VectorXd Zr;			// radar measurements
	Eigen::VectorXd br; 		// radar bias
	Eigen::VectorXd brSigmas; 	// characteristic standard deviations of radar bias white noise

	Eigen::VectorXd Zl;			// lidar measurements
	Eigen::VectorXd bl; 		// lidar bias
	Eigen::VectorXd blSigmas; 	// characteristic standard deviations of lidar bias white noise

	Eigen::MatrixXd errorPrDistanceGain;

	// Kalman parameters
	int n = 5; 			// number of states to track
	int nr = 3; 		// number of states measured by radar/lidar
	int na = 5;		 	// number of states received from AIS
	int nk = 19; 		// number of states in Kalman filter
	int nb = 13;
	double ha = 2;		// update interval of AIS
	double hr = 0.1;	// update interval of radar
	double hl = 0.1;	// update interval of lidar
	Eigen::VectorXd Za; 		// AIS measurements
	Eigen::VectorXd Za_prev; 	// Previous AIS measurements
	Eigen::VectorXd Xa_bar;
	Eigen::VectorXd Xa_hat;
	Eigen::VectorXd Xr_bar;
	Eigen::VectorXd Xr_hat;
	Eigen::VectorXd Xl_bar;
	Eigen::VectorXd Xl_hat;
	Eigen::MatrixXd Pa;
	Eigen::MatrixXd Pr;
	Eigen::MatrixXd Pl;
	Eigen::MatrixXd Pa_bar;
	Eigen::MatrixXd Pr_bar;
	Eigen::MatrixXd Pl_bar;
	Eigen::VectorXd fa_hat;
	Eigen::MatrixXd Tb;
	Eigen::MatrixXd Ha;
	Eigen::MatrixXd Hr;
	Eigen::MatrixXd Hl;
	Eigen::MatrixXd Ra;
	Eigen::MatrixXd Rr;
	Eigen::MatrixXd Rl;
	Eigen::MatrixXd Q;
	Eigen::MatrixXd E;
};


#endif // targetDetectionModule_H