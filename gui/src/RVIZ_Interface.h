#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H

#include "../environment/src/gpsTools.h"

#include <ros/ros.h>
#include <string>

using namespace std;

class rvizInterface
{
public:
	rvizInterface(ros::NodeHandle *nh);
	void set_object(string objectID, gpsPoint3DOF position, double crossSection);
	void show_detected_target(	int targetID, 
								string objectDescriptor, 
								gpsPointStamped position,
								double SOG, 
								double crossSection);

private:
	gpsPoint mapOrigin;
	ros::Publisher objectPub;
	ros::Publisher mapPub;
};

#endif // RVIZ_INTERFACE_H
