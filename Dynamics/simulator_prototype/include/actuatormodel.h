#ifndef ACTUATORMODEL_H
#define ACTUATORMODEL_H
#include "ros/ros.h"
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 3, 2> Matrix32;
typedef Matrix<double, 3, 3> Matrix33;
typedef Matrix<double, 7, 1> Vector7d;

class ActuatorModel{
public:
	ActuatorModel();
	~ActuatorModel();
	void initializeActuatorModel(double _K, double _K_b, double _T_alpha, double _T_beta, double _T_n, double _T_b, double _l_x_1, double _l_x_2, double _l_y_1, double _l_y_2, double _l_b, double _dt, double _n_max, double _n_min, double _n_b_max, double _n_b_min, double _alpha_max);
	void getForcesAndMoments(Vector6d &_tau_control, Vector7d desired_actuator_states);
	Vector7d getActuatorState();

private:
	// Thruster states
	double alpha_1, alpha_2, beta_1, beta_2, n_1, n_2, n_b;

	// Desired thruster states
	double n_1_ref, n_2_ref, n_b_ref, alpha_1_ref, alpha_2_ref, beta_1_ref, beta_2_ref;

	// Thruster properties, including time constants and position
	double K, K_b, T_alpha, T_beta, T_n, T_b, l_x_1, l_x_2, l_y_1, l_y_2, l_b, n_min, n_max, n_b_min, n_b_max, alpha_max;

	// Step-time for solving the diff. equations, timer to ensure that when thrust_messages stop, the thrust is set to zero
	double dt;

	Vector6d tau_control;
	Matrix33 A;
	Matrix3d B;
	Vector3d N;

	void updateActuatorStates();

	void updateRPM();

	void updateNozzleAngle();

	void updateReverseDeflector();

	void calculateForcesAndMoments();

	//void publishActuatorLog();

	// Used for logging of actuator-data. Necessary as Matlab prefers the standard ROS-messages, custom messages is more tricky.
	//ros::Publisher thruster_publisher = actuator_handle.advertise<geometry_msgs::Twist>("log/actuatorData", 0);
};

#endif