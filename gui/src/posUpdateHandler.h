#ifndef POSUPDATEHANDLER_H
#define POSUPDATEHANDLER_H

#include <QObject>
#include <string>
#include <QThread>
#include <QTimer>
#include <QDebug>

#include "satelliteview.h"
#include "realtimeplot.h"

#include "/opt/ros/kinetic/include/ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "environment/obstacleUpdate.h"
#include "environment/obstacleCmd.h"
#include "simulator_messages/Gps.h"


class watchDog : public QThread
{
	Q_OBJECT
public:
	watchDog(ros::NodeHandle n, string obstacleID, int msec = 200, QThread *parent = 0);
	~watchDog();
	void kick();

private slots:
	void timoutHandler();

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	string obstID;
	QTimer *timer;
	int timeoutInterval;
	void run();
};


class posUpdateHandler : public QThread
{
    Q_OBJECT
public:
	posUpdateHandler(const posUpdateHandler& other);
	posUpdateHandler(ros::NodeHandle n, satelliteView *Sv, realtimePlot *hdngPlot, realtimePlot *velPlot);

private:
	QThread *WDthread;
	void run();
	map<string, watchDog*> obstWDs;
	satelliteView *sv;

	realtimePlot *headingPlot;
	realtimePlot *velocityPlot;

	ros::NodeHandle nh;
	ros::Subscriber obstUpdateSub;
	ros::Subscriber gpsSub;
	void obstUpdateParser(const environment::obstacleUpdate::ConstPtr& updateMsg);
	void gpsParser(const simulator_messages::Gps::ConstPtr& gpsMsg);
};


#endif // POSUPDATEHANDLER_H

