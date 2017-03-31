#ifndef MRU_H
#define MRU_H

#include "sensor.h"

class MRU : public Sensor {

public:
  MRU();
  ~MRU();

  // Publishes the IMU-data to a sensor-topic
  void publishMruData(Vector6d nu, Vector6d eta);

private:
	
};

#endif