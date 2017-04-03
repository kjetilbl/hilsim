#include "windsensor.h"

WindSensor::WindSensor(){}

WindSensor::~WindSensor(){}

void WindSensor::publishWindSensorData(double V, double beta){
	if(step == steps_per_data_output){
	    step=0;
	    geometry_msgs::Twist windSensorMessage;
	    windSensorMessage.linear.x = V;
	    windSensorMessage.angular.z = beta;
	    wind_sensor_pub.publish(windSensorMessage);
	}
	step++;
}
