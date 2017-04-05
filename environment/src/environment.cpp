#include "obstacleControl.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "obstacleControl");
	ros::NodeHandle nh;
	obstacleHandler myObstHandler(nh);
	myObstHandler.run();
	//obstacleControl(argc, argv);
	ROS_INFO("Here...");
	ros::Rate loop_rate(1);
	while(true)
	{
		loop_rate.sleep();
	}	
    return 0;
}
