#include "mru.h"

MRU::MRU() {}
MRU::~MRU() {}

void MRU::publishMruData(Vector6d nu, Vector6d eta, double latitude, double longitude) {
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


    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = longitude;
    odom.pose.pose.position.y = latitude;
    odom.pose.pose.position.z = eta(2);
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(eta(3), eta(4), eta(5));

    odom.twist.twist.linear.x = nu(0);
    odom.twist.twist.linear.y = nu(1);
    odom.twist.twist.linear.z = nu(2);
    odom.twist.twist.angular.x = nu(3);
    odom.twist.twist.angular.y = nu(4);
    odom.twist.twist.angular.z = nu(5);

    //publish the message
    odom_pub.publish(odom);
  }
  step++;  
}
