#ifndef OBSTACLECONTROL_H
#define OBSTACLECONTROL_H

#include <string>
#include <vector>

#include "gpsTools.h"
#include "simObject.h"

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QMainWindow>

#include "ros/ros.h"
#include "simulator_messages/obstacleCmd.h"

using namespace std;

class obstacleManager : public QThread
{
	Q_OBJECT
public:
	obstacleManager(ros::NodeHandle *n, QThread *parent = 0);
	~obstacleManager();

private:
	void run();
	void spawn_obstacles();
	void spawn_fixed_obstacle(gpsPoint3DOF eta, double size);
	gpsPoint mapOrigin;
	void get_origin_from_sim_params(ros::NodeHandle nh);
	void command_parser(const simulator_messages::obstacleCmd::ConstPtr& cmd);
	ros::NodeHandle *nh;
	ros::Subscriber cmdSub;
	QThread *simObjectsThread = NULL;
	vector<simObject*> simObjects = vector<simObject*>(0);
};



#endif // OBSTACLECONTROL_H

