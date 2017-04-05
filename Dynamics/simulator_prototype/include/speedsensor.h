#ifndef SPEED_H
#define SPEED_H

#include "sensor.h"
#include "ros/ros.h"

class SpeedSensor: public Sensor{
	public:
		SpeedSensor();
		~SpeedSensor();
		void publishSpeedSensorData(double u, double v);
	private:
};

#endif