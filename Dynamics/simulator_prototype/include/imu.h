#ifndef IMU_H
#define IMU_H

#include "sensor.h"

class IMU : public Sensor {

public:
  IMU();
  ~IMU();

  // Publishes the IMU-data to a sensor-topic
  void publishImuData(Vector6d nu_dot, Vector6d nu);

private:
  
};

#endif