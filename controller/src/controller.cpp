#include "controller.h"
Controller::Controller(){
	u_r = 3;
	k_p_speed = 70;
	k_i_speed = 8;
	k_d_speed = 30;
	psi_r = 0*(M_PI/180);
	psi_r_filtered = 0*(M_PI/180);
	psi=0;
	k_p_heading = 100;
	k_i_heading = 0;
	k_d_heading = 10;
	i_speed=0;
	i_heading=0;
	d_u=0;
	next_position.longitude = 0;
	next_position.latitude = 0;
}
Controller::~Controller(){
	
}
void Controller::step(){
	static int step = 0;
	//psi_r = wp_control.compass_bearing(position, next_position);
	calculateRPM();
	calculateNozzleAngle();
	publishActuatorMessage();
	step++;
	if(step>2){
		psi_r = M_PI/2;
	}
}
void Controller::setDT(double dt_){
	dt = dt_;
}
double Controller::getDT(){
	return dt;
}
void Controller::calculateRPM(){
	engine_rpm = k_p_speed*(u_r-u)+k_i_speed*i_speed-k_d_speed*d_u;
	i_speed+=(u_r-u)*0.1;
	saturate(100, engine_rpm);
	//std::cout << "Speed: " << u << ", Ref: " << u_r << ", RPM: " << engine_rpm << std::endl;
}
void Controller::calculateNozzleAngle(){
	psi_r_filtered = referenceFilter(psi_r_filtered, psi_r);
	engine_nozzle = k_p_heading*(psi_r_filtered-psi)+k_i_heading*i_heading-k_d_heading*heading_rate;
	i_heading+=(psi_r_filtered-psi)*0.1;
	saturate(100, engine_nozzle);
	//std::cout << "Heading: " << psi*(180.0/M_PI) << ", Ref: " << psi_r_filtered*(180.0/M_PI) << ", Angle: " << engine_nozzle << ", Rate: " << heading_rate << ", I: " << i_heading << std::endl;
}
void Controller::saturate(double max, double &element){
	if(element>max){
		element=max;
	}
	if(element<-max){
		element=-max;
	}
}
double Controller::referenceFilter(double value, double desired_value){
	double alpha = 0.98;
	double filtered_value = alpha*value+(1-alpha)*desired_value;
	return filtered_value;
}
void Controller::publishActuatorMessage(){
	simulator_messages::ActuatorMessage actuator_message;
	actuator_message.header.stamp = ros::Time::now();
	actuator_message.header.frame_id = "/controller";
	actuator_message.rightRPM = engine_rpm;
	actuator_message.leftRPM = engine_rpm;
	actuator_message.rightNozzle = engine_nozzle;
	actuator_message.leftNozzle = engine_nozzle;
	actuator_message.rightDeflector = 100;
	actuator_message.leftDeflector = 100;
	actuator_pub.publish(actuator_message);
}
void Controller::setGains(double k_p_, double k_i_, double k_d_){
	
}
void Controller::receiveMRUData(const nav_msgs::Odometry::ConstPtr &mru_msg){
	double last_u = u;
	position.longitude = mru_msg->pose.pose.position.x;
	position.latitude = mru_msg->pose.pose.position.y;
	u = mru_msg->twist.twist.linear.x;
	heading_rate = mru_msg->twist.twist.angular.z;
	d_u = (u-last_u)/0.1;
	tf::Pose pose;
	tf::poseMsgToTF(mru_msg->pose.pose, pose);
	psi = tf::getYaw(pose.getRotation());
	while(psi<0 || psi>2*M_PI){
		if(psi<0){
			psi+=2*M_PI;
		}
		if(psi>2*M_PI){
			psi-=2*M_PI;
		}
	}
	if(startup){
		psi_r = psi;
		psi_r_filtered = psi;
		startup = false;
	}
}