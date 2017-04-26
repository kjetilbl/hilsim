#ifndef OBSTACLECONTROL_H
#define OBSTACLECONTROL_H

#include <string>
#include <mutex>
#include <vector>

#include "navData.h"
#include "gpsTools.h"

#include <QTimer>
#include <QObject>
#include <QThread>
#include <QDebug>
#include <QMainWindow>

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "environment/obstacleUpdate.h"
#include "environment/obstacleCmd.h"

using namespace std;

class simObject;
class ship;

class obstacleHandler : public QThread
{
	Q_OBJECT
public:
	obstacleHandler(ros::NodeHandle nh, QThread *parent = 0);
	~obstacleHandler();
	void run();

private:
	gpsPoint mapOrigin;
	void get_origin_from_sim_params(ros::NodeHandle nh);
	void command_parser(const environment::obstacleCmd::ConstPtr& cmd);
	ship *testShip;
	ros::NodeHandle n;
	ros::Subscriber cmdSub;
	QThread *simObjectsThread = NULL;
	vector<simObject*> agents = vector<simObject*>(0);
};

class simObject : public QThread
{
	Q_OBJECT
public:
	simObject( const simObject& other );
	simObject( ros::NodeHandle nh, string obstID, double Longitude, double Latitude, double Psi, QThread *parent );
	~simObject();
	void set_position(double Longitude, double Latitude, double Psi);
	double get_longitude();
	double get_latitude();
	double get_heading();
	void initiate_pos_report_broadcast();
	tf::TransformBroadcaster rvizBroadcast = tf::TransformBroadcaster();

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
	double longitude;
	double latitude;
	double psi;
	ros::NodeHandle n;
	ros::Publisher posUpdatePub;
	ros::Subscriber cmdSub;

private slots:
	void publish_position_report();
	virtual void move() = 0;

};


class fixedObstacle : public simObject
{
public:
	fixedObstacle( ros::NodeHandle nh, string obstID = "NO_ID", double Longitude = 0, double Latitude = 0, double Psi = 0, QThread *parent = 0 );
	~fixedObstacle(){};

private:
	void move();
};




class activeSimObject : public simObject
{
	Q_OBJECT
public:
	activeSimObject( ros::NodeHandle nh, uint32_t mmsiNumber, double Longitude, double Latitude, double Psi, QThread *parent );

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

	QTimer *AIStimer = NULL;
	uint32_t MMSI;
	navStatus status;
	double ROT;
	double SOG;
	posAccuracy positionAccuracy;
	ros::Publisher AISpub;
};




class ship : public activeSimObject
{
	Q_OBJECT
public:
	ship( ros::NodeHandle nh, uint32_t mmsiNumber, double Longitude = 0, double Latitude = 0, double Psi = 0, QThread *parent = 0 );
	void initiatialize_move_timer();

private:
	void run();
	QTimer *moveTimer = NULL;
	uint16_t moveIntervalMs;

private slots:
	void move();
};


#endif // OBSTACLECONTROL_H

