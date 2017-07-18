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
#include "simulator_messages/obstacleUpdate.h"
#include "simulator_messages/obstacleCmd.h"
#include "simulator_messages/Gps.h"
#include "simulator_messages/detectedTarget.h"


class posUpdateHandler : public QThread
{
    Q_OBJECT
public:
	posUpdateHandler(const posUpdateHandler& other);
	posUpdateHandler(ros::NodeHandle *n, satelliteView *Sv, realtimePlot *hdngPlot, realtimePlot *velPlot);
	~posUpdateHandler();
	void run();

private:
	rvizInterface *rviz;
	satelliteView *sv;

	realtimePlot *headingPlot;
	realtimePlot *velocityPlot;

	ros::NodeHandle *nh;
	ros::Subscriber obstUpdateSub;
	ros::Subscriber gpsSub;
	ros::Subscriber detectedTargetSub;
	void obstUpdateParser(const simulator_messages::obstacleUpdate::ConstPtr& updateMsg);
	void gpsParser(const simulator_messages::Gps::ConstPtr& gpsMsg);
	void detectedTargetParser(const simulator_messages::detectedTarget::ConstPtr& dtMsg);
};


#endif // POSUPDATEHANDLER_H

