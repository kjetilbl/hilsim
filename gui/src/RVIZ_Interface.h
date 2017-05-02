#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H

#include "../environment/src/gpsTools.h"

#include <ros/ros.h>
#include <string>

using namespace std;

class rvizInterface
{
public:
	rvizInterface(ros::NodeHandle nh);
	void set_object(string objectID, gpsPoint3DOF position);

private:
	gpsPoint mapOrigin;
	ros::Publisher objectPub;
};

#endif // RVIZ_INTERFACE_H
