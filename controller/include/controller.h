#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "waypointController.h"
#include "nav_msgs/Odometry.h"
#include "simulator_messages/ActuatorMessage.h"
#include "tf/transform_datatypes.h"

class Controller{
public:
	Controller();
	~Controller();
	void step();
	double getDT();
	void setDT(double dt_);
	void setGains(double k_p_, double k_i_, double k_d_);
private:
	WaypointController wp_control;
	gpsPoint position, next_position;
	bool startup = true;
	ros::NodeHandle nh;
	ros::Subscriber mru_message_rec = nh.subscribe<nav_msgs::Odometry>("sensors/odom", 0, &Controller::receiveMRUData, this);
	ros::Publisher actuator_pub = nh.advertise<simulator_messages::ActuatorMessage>("input/actuators", 0);
	void saturate(double max_abs, double &element);
	void calculateRPM();
	void calculateNozzleAngle();
	void publishActuatorMessage();
	double referenceFilter(double value, double desired_value);
	void receiveMRUData(const nav_msgs::Odometry::ConstPtr &mru_msg);
	double dt, k_p_speed, k_i_speed, k_d_speed, k_p_heading, k_i_heading, k_d_heading;
	double i_speed, i_heading;
	double roll, pitch, psi, u, psi_r, u_r, d_u, heading_rate;
	double psi_r_filtered, u_r_filtered;
	double engine_rpm, engine_nozzle;
};

#endif