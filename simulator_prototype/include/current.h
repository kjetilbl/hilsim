#ifndef CURRENT_H
#define CURRENT_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "weather.h"

class Current : public Weather{
public:
	Current();
	~Current();
	void getData(double &V_in, double &beta_in);

private:
	ros::NodeHandle nh;
	ros::Publisher current_pub =
	  nh.advertise<geometry_msgs::Twist>("log/current", 0);
};


#endif