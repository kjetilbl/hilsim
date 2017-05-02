#include "vesselnode.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "hil_simulator_node");
  ros::start();
  ROS_INFO("Started HIL Simulator node.");
  VesselNode vessel_node;
  ros::Rate loopRate(1/ vessel_node.getDT());

  while (ros::ok()) {
    vessel_node.step();
    ros::spinOnce();
    loopRate.sleep();
  }

  ROS_INFO("Exiting HIL Simulator node");
  ros::shutdown();
  return 0;
}
