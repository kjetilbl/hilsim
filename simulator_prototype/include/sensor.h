#ifndef SENSOR_H
#define SENSOR_H

#include <cmath>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "simulator_messages/Gps.h"

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

class Sensor {
public:
  void setStepSize(double stepsize);

  void setFrequency(double _frequency);

  void setNoise();

protected:
  // To simplify modifications on the HIL interface, all publishers are declared here, and used in each individual sensor class.
  ros::NodeHandle sensor_handle;
  ros::Publisher gps_pub = sensor_handle.advertise<simulator_messages::Gps>("sensors/gps", 0);
  ros::Publisher gps_pub2 = sensor_handle.advertise<geometry_msgs::Twist>("sensors/gpsLog", 0);
  ros::Publisher mru_velocity_pub = sensor_handle.advertise<geometry_msgs::Twist>("sensors/mru/velocity", 0);
  ros::Publisher mru_position_pub = sensor_handle.advertise<geometry_msgs::Twist>("sensors/mru/position", 0);
  ros::Publisher imu_pub = sensor_handle.advertise<geometry_msgs::Twist>("sensors/imu", 0);
  ros::Publisher speed_sensor_pub = sensor_handle.advertise<geometry_msgs::Twist>("sensors/speedSensor", 0);

  double dt, frequency, steps_per_data_output, step;
};

#endif