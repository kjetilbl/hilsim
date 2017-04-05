#include "sensor.h"

void Sensor::setFrequency(double _frequency){
  frequency = _frequency;
  if(frequency>int(1/dt)){
    ROS_INFO("Illegal combination of stepsize and sensor output frequency found, lowering sensor output frequency.");
    steps_per_data_output = 1;
  }else{
    steps_per_data_output = int((1/dt)/frequency);
  }
  step=1;
} 

void Sensor::setStepSize(double stepsize) { dt = stepsize; }
