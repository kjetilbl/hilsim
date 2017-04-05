#ifndef WINDSENSOR_H
#define WINDSENSOR_H

#include "sensor.h"
#include "ros/ros.h"

class WindSensor: public Sensor{
	public:
		WindSensor();
		~WindSensor();
		void publishWindSensorData(double V, double beta);
	private:
};

#endif