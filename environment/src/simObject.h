#ifndef SIMOBJECT_H
#define SIMOBJECT_H

#include <string>
#include <mutex>
#include <vector>

#include <QObject>
#include <QThread>
#include <QTimer>

#include "navData.h"
#include "gpsTools.h"

#include "ros/ros.h"
#include "environment/obstacleUpdate.h"
#include "environment/obstacleCmd.h"


using namespace std;

class simObject : public QThread
{
	Q_OBJECT
public:
	simObject( const simObject& other );
	simObject( ros::NodeHandle nh, string obstID, gpsPoint3DOF eta0, QThread *parent );
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

private:
	QTimer *posReportTimer = NULL;
	mutex m;
	bool stop = false;
	bool running = false;
	gpsPoint3DOF eta;
	ros::NodeHandle n;
	ros::Publisher posUpdatePub;
	ros::Subscriber cmdSub;

private slots:
	void publish_position_report();

};


class fixedObstacle : public simObject
{
public:
	fixedObstacle( ros::NodeHandle nh, gpsPoint3DOF eta0, QThread *parent = 0 );
	~fixedObstacle(){};

private:
	void run();
	static int IDiterator;
};




class aisUser : public simObject
{
	Q_OBJECT
public:
	aisUser( ros::NodeHandle nh, gpsPoint3DOF eta0, QThread *parent );

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


private:
	mutex activeObjMutex;

	static int IDiterator;
	QTimer *AIStimer = NULL;
	uint32_t MMSI;
	navStatus status;
	double ROT;
	double SOG;
	posAccuracy positionAccuracy;
	ros::Publisher AISpub;
};




class ship : public aisUser
{
	Q_OBJECT
public:
	ship( ros::NodeHandle nh, gpsPoint3DOF eta0, QThread *parent = 0 );
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