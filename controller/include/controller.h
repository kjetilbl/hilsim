#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

class Controller{
public:
	Controller();
	~Controller();
	void step();
	double getDT();
	void setDT(double dt_);
	void setGains(double k_p_, double k_i_, double k_d_);
private:
	ros::NodeHandle sensor_handle;
	ros::Subscriber mru_message_rec = sensor_handle.subscribe<nav_msgs::Odometry>("sensors/odom", 0, &Controller::receiveMRUData, this);
	void receiveMRUData(const nav_msgs::Odometry::ConstPtr &mru_msg);
	double dt, k_p, k_i, k_d;
	double i;
	double roll, pitch, yaw, u, psi_r, u_r;
};

#endif