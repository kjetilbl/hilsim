#include "vesselnode.h"


VesselNode::VesselNode(){
	dt = getDT();
	actuator_positions = vessel.getActuatorPositions();
	initializeActuatorMarkers();
}
VesselNode::~VesselNode(){

}

void VesselNode::step(){
	if(!paused){
		time_since_last_message += dt;
		publishTrace();
		if(time_since_last_message>1){
			tau_control << 0, 0, 0, 0, 0, 0;
		}
		vessel.setThrust(tau_control);
		publishActuatorMarkers();
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

void VesselNode::publishTrace(){
	static unsigned int trace_max_size = 1000;
	static int step = 0;
	static double simulator_frequency = 1.0/dt;
	static double trace_frequency = 5; 
	if(step*trace_frequency >= simulator_frequency){
		step=0;
		geometry_msgs::PoseStamped pose;
		trace.header.stamp = ros::Time::now();
		trace.header.frame_id = "map";
		pose.pose.position.x = eta[0];
		pose.pose.position.y = -eta[1];
		pose.pose.position.z = -eta[2];
		trace_log.push_front(pose);
		if(trace_log.size()>trace_max_size){			
				trace_log.resize(trace_max_size);
		}	
		trace.poses.resize(trace_log.size());
		for(unsigned int i=0; i<trace_log.size(); i++){
			trace.poses[i] = trace_log[i];
		}
		trace_pub.publish(trace);
	}

	step++;
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

void VesselNode::initializeActuatorMarkers(){
	actuator_1_marker.header.frame_id = tf_name;
    actuator_1_marker.header.stamp = ros::Time::now();
	actuator_2_marker.header.frame_id = tf_name;
    actuator_2_marker.header.stamp = ros::Time::now();
    actuator_bow_marker.header.frame_id = tf_name;
    actuator_bow_marker.header.stamp = ros::Time::now();

    actuator_1_marker.ns = "Actuator 1 Visualization";
    actuator_1_marker.id = 0; 
	actuator_2_marker.ns = "Actuator 2 Visualization";
    actuator_2_marker.id = 1; 
    actuator_bow_marker.ns = "Actuator bow Visualization";
    actuator_bow_marker.id = 3; 
    Vector7d actuator_states = vessel.actuators.getActuatorState();

    actuator_1_marker.type = visualization_msgs::Marker::ARROW;
 	actuator_2_marker.type = visualization_msgs::Marker::ARROW;
 	actuator_bow_marker.type = visualization_msgs::Marker::ARROW;

    actuator_1_marker.action = visualization_msgs::Marker::ADD;
	actuator_2_marker.action = visualization_msgs::Marker::ADD;
	actuator_bow_marker.action = visualization_msgs::Marker::ADD;

    actuator_1_marker.pose.position.x = actuator_positions(0);
    actuator_1_marker.pose.position.y = actuator_positions(1);
    actuator_1_marker.pose.position.z = 0;
    actuator_1_marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);

	actuator_2_marker.pose.position.x = actuator_positions(2);
    actuator_2_marker.pose.position.y = actuator_positions(3);
    actuator_2_marker.pose.position.z = 0;
    actuator_2_marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);

    actuator_bow_marker.pose.position.x = actuator_positions(4);
    actuator_bow_marker.pose.position.y = 0;
    actuator_bow_marker.pose.position.z = 0;
    actuator_bow_marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);

    double L_pp = vessel.getLength();
    actuator_1_marker.scale.x = L_pp*(0.03 + 0.2*(actuator_states(1)));
    actuator_1_marker.scale.y = L_pp*0.02;
    actuator_1_marker.scale.z = L_pp*0.02;

	actuator_2_marker.scale.x = L_pp*(0.03 + 0.2*(actuator_states(0)));
    actuator_2_marker.scale.y = L_pp*0.02;
    actuator_2_marker.scale.z = L_pp*0.02;

    actuator_bow_marker.scale.x = 0;
    actuator_bow_marker.scale.y = L_pp*0.02;
    actuator_bow_marker.scale.z = L_pp*0.02;

    actuator_1_marker.color.r = 1.0f;
    actuator_1_marker.color.g = 0.1f;
    actuator_1_marker.color.b = 0.1f;
    actuator_1_marker.color.a = 1.0;

	actuator_2_marker.color.r = 0.1f;
    actuator_2_marker.color.g = 1.0f;
    actuator_2_marker.color.b = 0.1f;
    actuator_2_marker.color.a = 1.0;

    actuator_bow_marker.color.r = 0.1f;
    actuator_bow_marker.color.g = 0.1f;
    actuator_bow_marker.color.b = 1.0f;
    actuator_bow_marker.color.a = 1.0;


    actuator_1_marker.lifetime = ros::Duration();
	actuator_2_marker.lifetime = ros::Duration();
	actuator_bow_marker.lifetime = ros::Duration();
}

void VesselNode::publishActuatorMarkers(){
    actuator_1_marker.header.stamp = ros::Time::now();
    actuator_2_marker.header.stamp = ros::Time::now();
    actuator_bow_marker.header.stamp = ros::Time::now();

    Vector7d actuator_states = vessel.actuators.getActuatorState();

    actuator_1_marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI - actuator_states(3));
    actuator_2_marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI - actuator_states(2));

    if(actuator_states(6)<0){
    	actuator_bow_marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
    }else{
    	actuator_bow_marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/2);
    }
    //std::cout << actuator_bow_marker.pose.orientation << std::endl;
    
    double L_pp = vessel.getLength();

    actuator_1_marker.scale.x = L_pp*(0.03 + 0.2*(actuator_states(1)));
	actuator_2_marker.scale.x = L_pp*(0.03 + 0.2*(actuator_states(0)));
	actuator_bow_marker.scale.x = L_pp*(0.2*(actuator_states(6)));

	marker_pub.publish(actuator_1_marker);
	marker_pub.publish(actuator_2_marker);
	marker_pub.publish(actuator_bow_marker);
}

void VesselNode::receiveForcesAndMoments(const geometry_msgs::Twist::ConstPtr &thrust_msg){
	tau_control << thrust_msg->linear.x, thrust_msg->linear.y, thrust_msg->linear.z, thrust_msg->angular.x, thrust_msg->angular.y, thrust_msg->angular.z;
	time_since_last_message = 0;
}

void VesselNode::receiveActuatorInfo(const simulator_messages::ActuatorMessage::ConstPtr &actuator_msg){
	desired_actuator_states << actuator_msg->rightRPM, actuator_msg->leftRPM, actuator_msg->rightNozzle, actuator_msg->leftNozzle, actuator_msg->rightDeflector, actuator_msg->leftDeflector, actuator_msg->bowThruster;
	vessel.actuators.getForcesAndMoments(tau_control, desired_actuator_states);
	time_since_last_message = 0;
}
