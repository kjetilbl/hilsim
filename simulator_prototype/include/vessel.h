#ifndef VESSEL_H
#define VESSEL_H
#include "ros/ros.h"
#include <Eigen/Dense>
#include <math.h>
#include "solver.h"
#include "sensor.h"
#include "mru.h"
#include "imu.h"
#include "gps.h"
#include "wind.h"
#include "current.h"
#include "actuatormodel.h"
#include "speedsensor.h"

using namespace Eigen;
typedef Matrix<double, 1, 2> Vector2dH;
typedef Matrix<double, 1, 3> Vector3dH;
typedef Matrix<double, 4, 1> Vector4d;
typedef Matrix<double, 1, 4> Vector4dH;
typedef Matrix<double, 5, 1> Vector5d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 4, 4> Matrix4d;
typedef Matrix<double, 5, 5> Matrix5d;
typedef Matrix<double, 6, 6> Matrix6d;

class Vessel {

public:
  Vessel();
  ~Vessel();

  // Returns the tau_control vector, used for testing the
  // receiveThrust()-function.
  Vector6d getThrust();

  void setThrust(Vector6d tau_in);

  // Reads vessel parameters from a <VesselName>.yaml-file, which should be
  // loaded through a launch-file. This ensures that different vessels can
  // easily be simulated.
  bool readParameters(ros::NodeHandle nh);

  // Fills in the loaded parameters in the M_rb, M_a, M, M_inv and G matrices,
  // as these are constant.
  void initializeMatrices();

  void initializeStateVectors();

  void initializeSensors();

  // Performs one step in the simulation
  void step();

  // Returns stepsize used in solver.
  double getDT();

  // Sets new states for the vessel. Could be useful for launching the vessel to
  // start at a given position in a map.
  void setState(Vector6d eta, Vector6d nu);

  void getState(Vector6d &eta, Vector6d &nu);

  void setGpsCoordinates(double _lat, double _long);


  double M_det;

  ActuatorModel actuators;

private:
  NumericalSolver solver;

  // Sensors on the vessel
  IMU imu;
  GPS gps;
  MRU mru;
  SpeedSensor speedSensor;

  Current current;
  Wind wind;
  double current_speed, current_direction;
  double wind_speed, wind_direction;
  double A_Fw, A_Lw, s_H, s_L, CD_t, CD_l_af_0, CD_l_af_pi, delta, kappa;
  void calculateEnvironmentalForces();
  void calculateCurrentForces();
  void calculateWindForces();
  void getRelativeWindParameters(double &speed_ned, double &direction_ned);

  // A set of state vectors, relative to different coordinate frames.
  Vector6d eta, nu, nu_r, nu_c_b, nu_c_n, nu_dot, delta_nu, mu;
  // Velocity in NED, used for GPS-position.
  Vector6d nu_n;
  // A set of different force vectors, used to represent forces from thrusters
  // and environmental forces
  Vector6d tau_control, tau_wind, tau_current, tau_waves, tau_total;

  // Necessary matrices to model the vessel
  Matrix6d M_rb, M_a, M, M_inv, C, C_a, C_rb, D, J, G;

  // Fluid memory matrices. Large system, quite heavy on calculation
  Matrix4d A_11, A_22, A_33, A_55, A_66, A_53, A_35;
  Matrix3d A_44, A_24, A_42;
  Matrix2d A_26, A_46, A_62, A_64;

  Vector4d B_11, B_22, B_33, B_55, B_66, B_53, B_35;
  Vector3d B_44, B_24, B_42;
  Vector2d B_26, B_46, B_62, B_64;

  Vector4dH C_11, C_22, C_33, C_55, C_66, C_53, C_35;
  Vector3dH C_44, C_24, C_42;
  Vector2dH C_26, C_46, C_62, C_64;

  Vector4d x_11, x_22, x_33, x_55, x_66, x_53, x_35;
  Vector3d x_44, x_24, x_42;
  Vector2d x_26, x_46, x_62, x_64;

  // Step size for solver
  double dt;

  void publishSensorData();

  void initializeFluidMemoryMatrices();

  void calculateFluidMemoryEffects();

  void calculateNonlinearSurge();

  void calculateCrossFlowDrag();
  // Receives thrust messages
  //void receiveThrust(const geometry_msgs::Twist::ConstPtr &thrust_msg);

  // Mass and inertia
  double m_11, m_12, m_13, m_14, m_15, m_16, m_21, m_22, m_23, m_24, m_25, m_26,
      m_31, m_32, m_33, m_34, m_35, m_36, m_41, m_42, m_43, m_44, m_45, m_46,
      m_51, m_52, m_53, m_54, m_55, m_56, m_61, m_62, m_63, m_64, m_65, m_66;

  // Added mass
  double X_u_dot, X_v_dot, X_w_dot, X_p_dot, X_q_dot, X_r_dot, Y_u_dot, Y_v_dot,
      Y_w_dot, Y_p_dot, Y_q_dot, Y_r_dot, Z_u_dot, Z_v_dot, Z_w_dot, Z_p_dot,
      Z_q_dot, Z_r_dot, K_u_dot, K_v_dot, K_w_dot, K_p_dot, K_q_dot, K_r_dot,
      M_u_dot, M_v_dot, M_w_dot, M_p_dot, M_q_dot, M_r_dot, N_u_dot, N_v_dot,
      N_w_dot, N_p_dot, N_q_dot, N_r_dot;

  double C_rb_26, C_rb_35, C_rb_46, C_a_26, C_a_35, C_a_46, C_a_55, C_a_66;

  // Center of gravity
  double x_g, y_g, z_g;

  // Restoring forces
  double Z_z, Z_theta, K_phi, M_z, M_theta;

  // Damping parameters
  double X_u, X_v, Y_v, Y_r, N_v, N_r, K_p, Z_w, M_q, M_r, K_r, Z_q;
  double X_uu, X_uuu, X_vv, X_vvv, Y_vv, Y_vvv, Y_rr, Y_rrr, Y_rv, Y_vr, N_rr,
      N_rrr, N_vv, N_vvv, N_rv, N_vr, M_qq;

  // Other
  double K_thruster, T_n, T_alpha, T_beta, n_min, n_max, alpha_max, l_y_1, l_y_2, l_x_1, l_x_2, L_pp, C_d_2d, T, X_uu_c;
  double surge_max;

  // Update frequency for onboard sensors.
  double gps_frequency, mru_frequency, imu_frequency, speed_sensor_frequency;

  // Starting position in lat/long
  double start_latitude, start_longitude;

  // The damping, coriolis and rotation matrix are dependent of the vessel
  // state, and needs to be updated for each step
  void updateMatrices();

  // Contains the kinematics, eta_dot = J(eta)*nu
  Vector6d etaFunction(Vector6d nu_in);

  // Contains the kinetics equation, nu_dot = -M_inv*(C*nu+D*nu+G*eta-tau_total)
  Vector6d nuFunction(Vector6d nu_in);

  // Uses the solver and etaFunction() to calculate next eta
  void calculateNextEta();

  // Uses the solver and nuFunction() to calculate next nu
  void calculateNextNu();
};

#endif
