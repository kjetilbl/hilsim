#ifndef OBSTACLEINTERFACE_H
#define OBSTACLEINTERFACE_H

#include <QGroupBox>
#include <QPushButton>
#include <QObject>
#include <string>
#include <thread>
#include <QThread>
#include <QTimer>
#include <QDebug>

#include "satelliteview.h"

#include "/opt/ros/kinetic/include/ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "environment/obstacleUpdate.h"
#include "environment/obstacleCmd.h"
#include "simulator_messages/Gps.h"


class obstacleInterface : public QObject
{
	Q_OBJECT
public:
	obstacleInterface(){};
	obstacleInterface(ros::NodeHandle nh, QGroupBox *interfaceWindow, satelliteView *Sv);
	void requestNewObstacle(double x, double y, double psi);

private slots:
	void handleSpawnButton();

private:
	//QThread *updateHandlingThread;
	//posUpdateHandler *myposUpdateHandler;
	satelliteView *sv;
	ros::Publisher cmdPub;
	QPushButton *spawnObstacleButton;
};


#endif // OBSTACLEINTERFACE_H

