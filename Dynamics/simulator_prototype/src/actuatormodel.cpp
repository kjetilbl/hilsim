#include "actuatormodel.h"

void ActuatorModel::initializeActuatorModel(double _K, double _K_b, double _T_alpha, double _T_beta, double _T_n, double _T_b, double _l_x_1, double _l_x_2, double _l_y_1, double _l_y_2, double _l_b, double _dt, double _n_max, double _n_min, double _n_b_max, double _n_b_min, double _alpha_max){ 
	K = _K;
	K_b = _K_b;
	T_alpha = _T_alpha;
	T_beta = _T_beta;
	T_n = _T_n;
	T_b = _T_b;
	l_x_1 = _l_x_1;
	l_x_2 = _l_x_2;
	l_y_1 = _l_y_1;
	l_y_2 = _l_y_2;
	l_b = _l_b;
	alpha_1 = 0;
	alpha_2 = 0;
	dt = _dt;
	n_min = _n_min;
	n_max = _n_max;
	n_b_min = _n_b_min;
	n_b_max = _n_b_max;
	alpha_max = _alpha_max;
}

ActuatorModel::ActuatorModel(){
	tau_control << 0,0,0,0,0,0;
}

ActuatorModel::~ActuatorModel(){

}

Vector7d ActuatorModel::getActuatorState(){
	Vector7d actuator_state;
	actuator_state << n_1/n_max, n_2/n_max, alpha_1, alpha_2, beta_1, beta_2, n_b/n_max;
	return actuator_state;
}

void ActuatorModel::calculateForcesAndMoments(){
	A << 	cos(alpha_1)*beta_1,					cos(alpha_2)*beta_2,					0,
			sin(alpha_1)*beta_1, 					sin(alpha_2)*beta_2,					1,
			l_x_1*sin(alpha_1)-l_y_1*cos(alpha_1),	l_x_2*sin(alpha_2)-l_y_2*cos(alpha_2),	l_b;
	
	B << 	K, 	0, 	0,
			0, 	K, 	0,
			0,	0, 	K_b;
	
	N << 	std::abs(n_1)*n_1, 	std::abs(n_2)*n_2, 	std::abs(n_b)*n_b;

	Vector3d tau_3DOF = A*B*N;
	tau_control << tau_3DOF(0), tau_3DOF(1), 0, 0, 0, tau_3DOF(2);
}

void ActuatorModel::updateRPM(){
	n_1 = n_1+dt*((-1/T_n)*(n_1-n_1_ref));
	n_2 = n_2+dt*((-1/T_n)*(n_2-n_2_ref));
	n_b = n_b+dt*((-1/T_b)*(n_b-n_b_ref));
}

void ActuatorModel::updateNozzleAngle(){
	alpha_1 = alpha_1+dt*((-1/T_alpha)*(alpha_1-alpha_1_ref));
	alpha_2 = alpha_2+dt*((-1/T_alpha)*(alpha_2-alpha_2_ref));
}

void ActuatorModel::updateReverseDeflector(){
	beta_1 = beta_1+dt*((-1/T_beta)*(beta_1-beta_1_ref));
	beta_2 = beta_2+dt*((-1/T_beta)*(beta_2-beta_2_ref));
}

void ActuatorModel::getForcesAndMoments(Vector6d &_tau_control, Vector7d desired_actuator_states){
	n_1_ref = n_min+((n_max-n_min)/100)*desired_actuator_states(0);
	n_2_ref = n_min+((n_max-n_min)/100)*desired_actuator_states(1);
	n_b_ref = ((n_b_max-n_b_min)/100)*desired_actuator_states(6);
	alpha_1_ref = (-alpha_max*M_PI/180)*desired_actuator_states(2)/100;
	alpha_2_ref = (-alpha_max*M_PI/180)*desired_actuator_states(3)/100;
	beta_1_ref = desired_actuator_states(4)/100;
	beta_2_ref = desired_actuator_states(5)/100;
	updateRPM();
	updateNozzleAngle();
	updateReverseDeflector();
	calculateForcesAndMoments();
	_tau_control = tau_control;
}

// Used for logging of actuator-data. Necessary as Matlab prefers the standard ROS-messages, custom messages is more tricky.
/*void ActuatorModel::publishActuatorLog(){
	geometry_msgs::Twist thrust_msg;
	thrust_msg.linear.x = n_1;
	thrust_msg.linear.y = alpha_1;
	thrust_msg.linear.z = beta_1;
	thrust_msg.angular.x = n_1_ref;
	thrust_msg.angular.y = alpha_1_ref;
	thrust_msg.angular.z = beta_1_ref;
	thruster_publisher.publish(thrust_msg);
}*/
