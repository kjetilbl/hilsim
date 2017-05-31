#ifndef VESSELNODE_H
#define VESSELNODE_H

#include "ros/ros.h"
#include "simulator_messages/Environment.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Path.h"
#include "simulator_messages/ActuatorMessage.h"
#include "vessel.h"
#include <queue>

class VesselNode{
public:
	VesselNode();
	~VesselNode();
	Vessel vessel;
	void step();
	double getDT();
private:
	bool paused = false;
	Vector6d eta, nu, desired_actuator_states;
	Vector6d tau_control = Vector6d::Zero();
	Vector4d actuator_positions;

	void receiveEnvironmentMessage(const simulator_messages::Environment::ConstPtr &environment_msg);

	void logInfo();

	void publishState();

	void publishTrace();

	void receiveForcesAndMoments(const geometry_msgs::Twist::ConstPtr &thrust_msg);

	void receiveActuatorInfo(const simulator_messages::ActuatorMessage::ConstPtr &actuator_msg);

	void initializeActuatorMarkers();

	void publishActuatorMarkers();

	geometry_msgs::Twist vectorToGeometryMsg(Vector6d vector_in);

	nav_msgs::Path trace;
	std::deque<geometry_msgs::PoseStamped> trace_log;
	ros::NodeHandle trace_handle;
	ros::Publisher trace_pub = trace_handle.advertise<nav_msgs::Path>("log/trace", 1);

	double time_since_last_message, dt;

	visualization_msgs::Marker actuator_1_marker;
	visualization_msgs::Marker actuator_2_marker;

	tf::TransformBroadcaster tf = tf::TransformBroadcaster();
  	std::string tf_name = "simulated_vessel";
  	ros::NodeHandle log_handle;
	ros::Publisher vel_pub =
	  log_handle.advertise<geometry_msgs::Twist>("log/velocity", 0);
	ros::Publisher state_pub =
	  log_handle.advertise<geometry_msgs::Twist>("log/state", 0);
	ros::Publisher thrust_pub =
	  log_handle.advertise<geometry_msgs::Twist>("log/thrust", 0);

	ros::NodeHandle actuator_handle;
	ros::Publisher marker_pub = actuator_handle.advertise<visualization_msgs::Marker>("actuator_markers", 1);
	// Used to receive info about the desired actuator states, update the simulated states, and get the corresponding forces and moments
	ros::Subscriber actuator_message_rec = actuator_handle.subscribe<simulator_messages::ActuatorMessage>(
	  "input/actuators", 0, &VesselNode::receiveActuatorInfo, this);

	// Used to receive forces and moments directly, bypassing the actuator-model.
	ros::Subscriber force_rec = actuator_handle.subscribe<geometry_msgs::Twist>(
	  "input/thrust", 0, &VesselNode::receiveForcesAndMoments, this);

	ros::NodeHandle environment_comm_handle;
	//ros::Subscriber environment_msg_rec = environment_comm_handle.subscribe<simulator_messages::Environment>("input/environment_communication", 0, &VesselNode::receiveEnvironmentMessage, this);
};


#endif
