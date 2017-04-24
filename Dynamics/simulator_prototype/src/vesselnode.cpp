#include "vesselnode.h"


VesselNode::VesselNode(){
	dt = getDT();
}
VesselNode::~VesselNode(){

}

void VesselNode::step(){
	if(!paused){
		time_since_last_message += dt;
		if(time_since_last_message>1){
			tau_control << 0, 0, 0, 0, 0, 0;
		}
		vessel.setThrust(tau_control);
		vessel.step();
		vessel.getState(eta, nu);
		logInfo();
		publishState();
	}else{

	}	
}

double VesselNode::getDT(){
	return vessel.getDT();
}

void VesselNode::publishState(){
	tf::Transform transform;
    transform.setOrigin(tf::Vector3(eta[0],-eta[1],-eta[2]));
    tf::Quaternion q;
    q.setRPY(-eta[3],-eta[4],-eta[5]);
    transform.setRotation(q);
    tf.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", tf_name));
}


void VesselNode::receiveEnvironmentMessage(const simulator_messages::Environment::ConstPtr &environment_msg){
	vessel.setGpsCoordinates(environment_msg->latitude, environment_msg->longitude);
	paused = environment_msg->paused;
	if(paused)
		ROS_INFO("Simulation paused from GUI...");
	if(!paused)
		ROS_INFO("Simulation started from GUI...");
}

void VesselNode::logInfo(){
	geometry_msgs::Twist velocity = vectorToGeometryMsg(nu);
	geometry_msgs::Twist state = vectorToGeometryMsg(eta);
	geometry_msgs::Twist thrust = vectorToGeometryMsg(tau_control);

	vel_pub.publish(velocity);

	state_pub.publish(state);

	thrust_pub.publish(thrust);
}

geometry_msgs::Twist VesselNode::vectorToGeometryMsg(Vector6d vector_in){
	geometry_msgs::Twist geometry_msg;
	geometry_msg.linear.x = vector_in(0);
	geometry_msg.linear.y = vector_in(1);
	geometry_msg.linear.z = vector_in(2);
	geometry_msg.angular.x = vector_in(3);
	geometry_msg.angular.y = vector_in(4);
	geometry_msg.angular.z = vector_in(5);
	return geometry_msg;
}

void VesselNode::receiveForcesAndMoments(const geometry_msgs::Twist::ConstPtr &thrust_msg){
	tau_control << thrust_msg->linear.x, thrust_msg->linear.y, thrust_msg->linear.z, thrust_msg->angular.x, thrust_msg->angular.y, thrust_msg->angular.z;
	time_since_last_message = 0;
}

void VesselNode::receiveActuatorInfo(const simulator_messages::ActuatorMessage::ConstPtr &actuator_msg){
	desired_actuator_states << actuator_msg->rightRPM, actuator_msg->leftRPM, actuator_msg->rightNozzle, actuator_msg->leftNozzle, actuator_msg->rightDeflector, actuator_msg->leftDeflector;
	vessel.actuators.getForcesAndMoments(tau_control, desired_actuator_states);
	time_since_last_message = 0;
}
