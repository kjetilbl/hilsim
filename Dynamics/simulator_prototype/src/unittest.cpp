#include <ros/ros.h>
#include "vessel.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>

struct pubSubHelper {
  int numRecMessages = 0;
  void callback(const geometry_msgs::Twist::ConstPtr &msg) { numRecMessages++; }
};

TEST(Vessel, parameterLoad) {
  ros::NodeHandle testHandle;
  Vessel testVessel;
  EXPECT_FALSE(testVessel.readParameters(testHandle));
}

TEST(Vessel, initializeMatrices) {
  Vessel testVessel;
  ros::NodeHandle testHandle;
  testVessel.readParameters(testHandle);
  testVessel.initializeMatrices();
  ASSERT_NE(testVessel.M_det, 0);
}

TEST(Vessel, setGetState) {
  Vector6d newState = Vector6d::Zero();
  Vector6d testState;
  testState << 1, 2, 3, 4, 5, 6;
  Vessel testVessel;
  testVessel.setState(newState, newState);
  testVessel.getState(testState, testState);
  EXPECT_EQ(newState, testState);
}

TEST(Vessel, getDT) { Vessel testVessel; }

TEST(Basic, publishSubscribe) {
  ros::NodeHandle nh;
  pubSubHelper h;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("testPub", 0);
  ros::Subscriber sub = nh.subscribe("testPub", 0, &pubSubHelper::callback, &h);
  EXPECT_EQ(pub.getNumSubscribers(), 1);
  EXPECT_EQ(sub.getNumPublishers(), 1);
  geometry_msgs::Twist msg;
  pub.publish(msg);
  ros::Rate delayTime(1 / 0.1);
  delayTime.sleep();
  ros::spinOnce();
  EXPECT_EQ(h.numRecMessages, 1);
}

TEST(Vessel, receiveThrust) {
  Vessel testVessel;
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("hil_sim/thrust", 0);
  geometry_msgs::Twist newThrust;
  double test_val = 123;
  newThrust.linear.x = test_val;
  pub.publish(newThrust);
  ros::Rate delayTime(1 / 0.1);
  delayTime.sleep();
  ros::spinOnce();
  Vector6d returnedThrust = testVessel.getThrust();
  EXPECT_EQ(returnedThrust(0), test_val);
  EXPECT_NE(returnedThrust(1), test_val);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "GTestNode");
  ros::start();
  testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}