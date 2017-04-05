#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "ps3.h"
#include "simulator_messages/ActuatorMessage.h"
#include <math.h>


class JoystickPublisher
{
public:
  JoystickPublisher();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  double left_thr, right_thr;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

}; 


JoystickPublisher::JoystickPublisher():
  left_thr(PS3_AXIS_STICK_LEFT_UPWARDS),
  right_thr(PS3_AXIS_STICK_RIGHT_UPWARDS)
{
  vel_pub_ = nh_.advertise<simulator_messages::ActuatorMessage>("input/actuators", 0);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 0, &JoystickPublisher::joyCallback, this);

}

void JoystickPublisher::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  simulator_messages::ActuatorMessage actuators;
  actuators.rightRPM = 100*(joy->axes[right_thr]);
  actuators.leftRPM = 100*(joy->axes[left_thr]);
  actuators.rightNozzle = 0;
  actuators.leftNozzle = 0;
  actuators.rightDeflector = 100;
  actuators.leftDeflector = 100;
  actuators.header.stamp = ros::Time::now();
  actuators.header.frame_id = "/jolner";
  vel_pub_.publish(actuators);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ps3_joy_thrusters");
	ROS_INFO("Started PS3 Jolner Controller node");
	JoystickPublisher joystick_publisher;
	

	ros::spin();
}
