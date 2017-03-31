#include "imu.h"

IMU::IMU() {}
IMU::~IMU() {}

void IMU::publishImuData(Vector6d nu_dot, Vector6d nu) {
	Vector6d imu_data;
	if(step == steps_per_data_output){
		step=0;
		geometry_msgs::Twist imuMessage;
		imuMessage.linear.x = imu_data(0);
		imuMessage.linear.y = imu_data(1);
		imuMessage.linear.z = imu_data(2);
		imuMessage.angular.x = imu_data(3);
		imuMessage.angular.y = imu_data(4);
		imuMessage.angular.z = imu_data(5);
		imu_pub.publish(imuMessage);
	}
	step++; 
}
