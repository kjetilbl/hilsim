#ifndef SIMOBJECT_H
#define SIMOBJECT_H

#include <string>
#include <mutex>
#include <vector>
#include "Eigen/Dense"

#include <QObject>
#include <QThread>
#include <QTimer>

#include "navData.h"
#include "gpsTools.h"

#include "ros/ros.h"
#include "environment/obstacleUpdate.h"
#include "environment/obstacleCmd.h"
#include "simulator_messages/AIS.h"


using namespace std;

class simObject : public QThread
{
	Q_OBJECT
public:
	simObject( const simObject& other );
	simObject( ros::NodeHandle *n, string obstID, gpsPoint3DOF eta0, double Size, QThread *parent );
	~simObject();
	void set_eta(gpsPoint3DOF newEta);
	gpsPoint3DOF get_eta();
	void initiate_pos_report_broadcast();

protected:
	virtual void run();
	void command_parser(const environment::obstacleCmd::ConstPtr& cmd);
	environment::obstacleUpdate make_position_update_msg();
	string objectDescriptor;
	string ID;
	ros::NodeHandle *nh;

private:
	QTimer *posReportTimer = NULL;
	mutex m;
	double size;
	bool stop = false;
	bool running = false;
	gpsPoint3DOF eta;
	ros::Publisher posUpdatePub;
	ros::Subscriber cmdSub;

private slots:
	void publish_position_report();

};


class fixedObstacle : public simObject
{
public:
	fixedObstacle( ros::NodeHandle *n, gpsPoint3DOF eta0, double Size, QThread *parent = 0 );
	~fixedObstacle(){};

private:
	void run();
	static int IDiterator;
};




class aisUser : public simObject
{
	Q_OBJECT
public:
	aisUser( ros::NodeHandle *n, gpsPoint3DOF eta0, double Size, QThread *parent );

private slots:
	void broadcast_AIS_msg();

protected:
	void set_MMSI( uint32_t ID);
	uint32_t get_MMSI();
	void set_status(navStatus newStatus);
	navStatus get_status();
	void set_ROT(double rot);
	double get_ROT();
	void set_SOG(double sog);
	double get_SOG();
	void set_pos_accuracy(posAccuracy accuracy);
	posAccuracy get_pos_accuracy();
	void initiate_AIS_broadcast(uint16_t intervalMs);
	void pause_AIS_broadcast(){ AISenabled = false; }
	void continue_AIS_broadcast(){ AISenabled =true; }


private:
	bool read_AIS_config();
	Eigen::VectorXd get_estimated_nav_parameters();
	mutex activeObjMutex;

	static int IDiterator;
	bool AISenabled = true;
	uint16_t AISinterval;
	QTimer *AIStimer = NULL;
	uint32_t MMSI;
	navStatus status;
	double ROT;
	double SOG;
	posAccuracy positionAccuracy;
	ros::Publisher AISpub;

	// Error parameters
	bool firstTimeErrorCalc = true;
	QTime lastErrorCalcTime;
	bool updatedParameters = false;
	Eigen::VectorXd b; // bias
	Eigen::MatrixXd T; // continous-time bias system matrix
	Eigen::MatrixXd Td; // discrete-time bias system matrix
	Eigen::VectorXd biasSigmas; // characteristic standard deviations of bias white noise
	Eigen::VectorXd measureSigmas; // characteristic standard deviations of measurement white noise
};




class ship : public aisUser
{
	Q_OBJECT
public:
	ship( ros::NodeHandle *n, gpsPoint3DOF eta0, double Size, QThread *parent = 0 );
	void add_waypoint(gpsPoint wp);

private:
	void run();
	QTimer *stepTimer = NULL;
	uint16_t stepIntervalMs;
	gpsPoint3DOF calculate_next_eta();
	vector<gpsPoint> waypoints;

private slots:
	virtual void step();
};


#endif // SIMOBJECT_H



/*
Spørsmål:
Er det faglig ukorrekt at eta inneholder long, lat og heading?

*/