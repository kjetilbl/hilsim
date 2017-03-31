#include "ros/ros.h"
#include "simulator_messages/ActuatorMessage.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <fstream>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "txt_to_actuator_info");
	ros::start();
	ROS_INFO("Started Odin actuator data node");
	
    //ros::Duration(5).sleep(); // Wait for RViz to start
    ros::Rate loopRate(10);
    simulator_messages::ActuatorMessage actuators;
    geometry_msgs::Twist actuators2;
    ros::NodeHandle nh_;
	ros::Publisher actuator_pub = nh_.advertise<simulator_messages::ActuatorMessage>("input/actuators", 0);
	ros::Publisher actuator_pub2 = nh_.advertise<geometry_msgs::Twist>("input/actuators2", 0);

	std::string fpath;
	nh_.getParam("path_name", fpath);
    std::vector<std::string> line;
    std::string mystr;
    std::ifstream myfile(fpath.c_str());
    if (myfile.is_open()){	
		while(myfile >> mystr)
		{
		   line.push_back(mystr);
		}
		ROS_INFO("Actuator data read.");
	}
	else{
		ROS_INFO("Cannot find input-file");
	}
	//std::cout << line.size() << std::endl;
	std::vector<double> leftRPM;
	std::vector<double> rightRPM;
	std::vector<double> leftNozzle;
	std::vector<double> rightNozzle;
	std::vector<double> leftBucket;
	std::vector<double> rightBucket;
	for(int i=0; i<line.size(); i++){
		if(i%6==0){
			leftRPM.push_back(atof(line[i].c_str()));
		}
		if(i%6==1){
			leftNozzle.push_back(atof(line[i].c_str()));
		}
		if(i%6==2){
			leftBucket.push_back(atof(line[i].c_str()));
		}
		if(i%6==3){
			rightRPM.push_back(atof(line[i].c_str()));
		}
		if(i%6==4){
			rightNozzle.push_back(atof(line[i].c_str()));
		}
		if(i%6==5){
			rightBucket.push_back(atof(line[i].c_str()));
		}
	}
	//std::cout << leftNozzle[3] << std::endl;
	int i=0;

    while(i<leftRPM.size() && ros::ok()){
    	actuators.header.stamp = ros::Time::now();
  		actuators.header.frame_id = "/odin";
  		actuators.rightRPM = rightRPM[i];
  		actuators.leftRPM = leftRPM[i];
  		actuators.rightNozzle = rightNozzle[i];
  		actuators.leftNozzle = leftNozzle[i];
  		actuators.rightDeflector = rightBucket[i];
  		actuators.leftDeflector = leftBucket[i];
  		actuator_pub.publish(actuators);

  		actuators2.linear.x = rightRPM[i];
  		actuators2.linear.y = leftRPM[i];
  		actuators2.linear.z = rightNozzle[i];
  		actuators2.angular.x = leftNozzle[i];
  		actuators2.angular.y = rightBucket[i];
  		actuators2.angular.z = leftBucket[i];
  		actuator_pub2.publish(actuators2);
  		i++;
    	loopRate.sleep();
    	ros::spinOnce();
    }
    ROS_INFO("Actuator input file ended.");

	
}
