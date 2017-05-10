#ifndef POSUPDATEHANDLER_H
#define POSUPDATEHANDLER_H

#include <QObject>
#include <string>
#include <QThread>
#include <QDebug>

#include "satelliteview.h"
#include "realtimeplot.h"
#include "RVIZ_Interface.h"

#include "/opt/ros/kinetic/include/ros/ros.h"
#include "environment/obstacleUpdate.h"
#include "environment/obstacleCmd.h"
#include "simulator_messages/Gps.h"


class posUpdateHandler : public QThread
{
    Q_OBJECT
public:
	posUpdateHandler(const posUpdateHandler& other);
	posUpdateHandler(ros::NodeHandle n, satelliteView *Sv, realtimePlot *hdngPlot, realtimePlot *velPlot);
	~posUpdateHandler();
	void run();

private:
	rvizInterface *rviz;
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

