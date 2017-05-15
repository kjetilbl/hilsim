#include "controller.h"
Controller::Controller(){

}
Controller::~Controller(){
	
}
void Controller::step(){

}
void Controller::setDT(double dt_){
	dt = dt_;
}
double Controller::getDT(){
	return dt;
}
void Controller::setGains(double k_p_, double k_i_, double k_d_){
	k_p = k_p_;
	k_i = k_i_;
	k_d = k_d_;
}
void Controller::receiveMRUData(const nav_msgs::Odometry::ConstPtr &mru_msg){
	u = mru_msg->twist.twist.linear.x;
	std::cout << "Speed: " << u << std::endl;
	//mru_msg->pose.pose.orientation.getRPY(roll, pitch, yaw);
}