#include "ros/ros.h"
#include "controller.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "controller");
	ros::start();
	ROS_INFO("Started controller node.");
	Controller controller;
	ros::Rate loopRate(1/ 0.1);//controller.getDT());

	while (ros::ok()) {
		controller.step();
		ros::spinOnce();
		loopRate.sleep();
	}

	ROS_INFO("Exiting controller node");
	ros::shutdown();
	return 0;
}