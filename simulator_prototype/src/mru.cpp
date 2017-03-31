#include "mru.h"

MRU::MRU() {}
MRU::~MRU() {}

void MRU::publishMruData(Vector6d nu, Vector6d eta) {
  if(step == steps_per_data_output){
    step = 0;
    geometry_msgs::Twist mruVelocityMessage;
    geometry_msgs::Twist mruPositionMessage;
    mruPositionMessage.linear.x = eta(0);
    mruPositionMessage.linear.y = eta(1);
    mruPositionMessage.linear.z = eta(2);
    mruPositionMessage.angular.x = eta(3);
    mruPositionMessage.angular.y = eta(4);
    mruPositionMessage.angular.z = eta(5);

    mruVelocityMessage.linear.x = nu(0);
    mruVelocityMessage.linear.y = nu(1);
    mruVelocityMessage.linear.z = nu(2);
    mruVelocityMessage.angular.x = nu(3);
    mruVelocityMessage.angular.y = nu(4);
    mruVelocityMessage.angular.z = nu(5);

    mru_position_pub.publish(mruPositionMessage);
    mru_velocity_pub.publish(mruVelocityMessage);
  }
  step++;  
}
