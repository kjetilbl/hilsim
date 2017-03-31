#ifndef OBSTACLECONTROL_H
#define OBSTACLECONTROL_H

#include "ros/ros.h"
#include "environment/obstacleUpdate.h"
#include "environment/obstacleCmd.h"
#include <string>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <vector>

using namespace std;

void obstacleControl(int argc, char *argv[]);

class obstacleHandler
{
public:
	obstacleHandler(ros::NodeHandle nh);
	void run();

private:
	void cmdParser(const environment::obstacleCmd::ConstPtr& cmd);
	ros::NodeHandle n;
	ros::Subscriber cmdSub;
	vector<thread> agents = vector<thread>(0);
};

class obstacle
{
public:
	obstacle( const obstacle& other);
	obstacle(ros::NodeHandle nh, string obstID = "NO_ID", double X = 0, double Y = 0, double Psi = 0);
	void run();
private:
	void sendUpdateMsg();
	void cmdParser(const environment::obstacleCmd::ConstPtr& cmd);
	environment::obstacleUpdate getUpdateMsg();
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


#endif // OBSTACLECONTROL_H

