#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "ps3.h"
#include <math.h>


class JoystickPublisher
{
public:
  JoystickPublisher();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  double yaw, surge_fw, surge_bw, sway;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

}; 


JoystickPublisher::JoystickPublisher():
  yaw(PS3_AXIS_STICK_LEFT_LEFTWARDS),
  surge_fw(PS3_BUTTON_REAR_RIGHT_2),
  surge_bw(PS3_BUTTON_REAR_LEFT_2),
  sway(PS3_AXIS_STICK_RIGHT_LEFTWARDS)

{

  nh_.param("axis_yaw", yaw, yaw);
  nh_.param("axis_surge", surge_fw, surge_fw);
  nh_.param("axis_surge", surge_bw, surge_bw);
  nh_.param("axis_sway", sway, sway);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("input/thrust", 0);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 0, &JoystickPublisher::joyCallback, this);

}

void JoystickPublisher::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.linear.x = -10000*(joy->axes[surge_fw]-1)+10000*(joy->axes[surge_bw]-1);
  twist.linear.y = -10000*(joy->axes[sway]);
  twist.angular.z = -50000*(joy->axes[yaw]);
  vel_pub_.publish(twist);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ps3_thrust_input");
	ROS_INFO("Started PS3 Controller node");
	JoystickPublisher joystick_publisher;
	

	ros::spin();
}
