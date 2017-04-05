#include "speedsensor.h"

SpeedSensor::SpeedSensor(){}

SpeedSensor::~SpeedSensor(){}

void SpeedSensor::publishSpeedSensorData(double u, double v){
	if(step == steps_per_data_output){
	    step=0;
	    geometry_msgs::Twist speedSensorMessage;
	    speedSensorMessage.linear.x = u;
	    speedSensorMessage.linear.y = v;
	    speed_sensor_pub.publish(speedSensorMessage);
	}
	step++;
}
