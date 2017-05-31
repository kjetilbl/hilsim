#include "current.h"

Current::Current(){	
	mu_V = 1;
	mu_beta = 0.1;
	max_beta_deviation = 5;
	max_V_deviation = 0.2;
}
Current::~Current(){

}

void Current::getData(double &V_in, double &beta_in){
	if(mean_V==0){
		V_in = 0;
		V_total = 0;
		V = 0;
		beta = 0;
		beta_in = 0;
	}else{
		updateSpeed();
		updateDirection();
		V = maintainSaturation(V, 0, max_V_deviation);
		V_total = V+mean_V;
		V_in = V_total;
		beta = maintainSaturation(beta, 0, max_beta_deviation);
		beta_in = beta+mean_beta;
	}
	
	geometry_msgs::Twist current_msg;
	current_msg.linear.x = mean_V;
	current_msg.linear.y = V_total;
	current_msg.linear.z = V;
	current_msg.angular.y = mean_beta;
	current_msg.angular.z = beta;

	current_pub.publish(current_msg);
}
