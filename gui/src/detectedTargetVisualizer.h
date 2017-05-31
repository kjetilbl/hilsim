#ifndef DETECTEDTARGETVISUALIZER_H
#define DETECTEDTARGETVISUALIZER_H

#include <string>

#include <QObject>
#include <QThread>
#include <QDebug>

#include "../environment/src/gpsTools.h"
#include "RVIZ_Interface.h"

#include <ros/ros.h>
#include "simulator_messages/detectedTarget.h"

using namespace std;

class detectedTargetVisualizer
{
public:
	detectedTargetVisualizer(ros::NodeHandle n);
};

#endif // DETECTEDTARGETVISUALIZER_H
