#ifndef WIND_H
#define WIND_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "weather.h"

class Wind : public Weather {
public:
	Wind();
	~Wind();
	void getData(double &V_in, double &beta_in);
private:	
	double calculateWindSpeedAtHeight(double U_10, double z);
	void updateGustSpeed();
	double generateRandomGust();
	double V_gust, V_total;
	ros::NodeHandle nh;
	ros::Publisher wind_pub =
	  nh.advertise<geometry_msgs::Twist>("log/wind", 0);
};

#endif