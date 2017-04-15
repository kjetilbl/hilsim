#ifndef OBSTACLECONTROL_H
#define OBSTACLECONTROL_H

#include <string>
#include <mutex>
#include <vector>

#include <QThread>

#include "ros/ros.h"
#include "environment/obstacleUpdate.h"
#include "environment/obstacleCmd.h"

using namespace std;

class simObject;

class obstacleHandler
{
public:
	obstacleHandler(ros::NodeHandle nh);
	void run();

private:
	void command_parser(const environment::obstacleCmd::ConstPtr& cmd);
	ros::NodeHandle n;
	ros::Subscriber cmdSub;
	QThread *simObjectsThread = new QThread();
	vector<simObject*> agents = vector<simObject*>(0);
};

class simObject : public QThread
{
	Q_OBJECT
public:
	simObject( const simObject& other );
	simObject( ros::NodeHandle nh, string obstID, double X, double Y, double Psi, QThread *parent );
	~simObject(){};

protected:
	void run();
	virtual void move() = 0;
	void publish_position_report();
	void command_parser(const environment::obstacleCmd::ConstPtr& cmd);
	environment::obstacleUpdate make_position_update_msg();
	mutex m;
	bool stop = false;
	bool running = false;
	string ID;
	double x;
	double y;
	double psi;
	ros::NodeHandle n;
	ros::Publisher posUpdatePub;
	ros::Subscriber cmdSub;
};

class fixedObstacle : public simObject
{
public:
	fixedObstacle( ros::NodeHandle nh, string obstID = "NO_ID", double X = 0, double Y = 0, double Psi = 0, QThread *parent = 0 );
	~fixedObstacle(){};

private:
	void move();
};

class ship : public simObject
{
public:
	ship( ros::NodeHandle nh, string obstID = "NO_ID", double X = 0, double Y = 0, double Psi = 0, QThread *parent = 0 );

private:
	void move();
};


#endif // OBSTACLECONTROL_H

