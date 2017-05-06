#include "vessel.h"

Vessel::Vessel(){
	// On creation of new vessel
	initializeStateVectors();	
	ros::NodeHandle nh;
	readParameters(nh);
	K_wave << K_1_w, K_2_w, K_3_w, K_4_w, K_5_w, K_6_w;
	actuators.initializeActuatorModel(K_thruster, T_alpha, T_beta, T_n, l_x_1, l_x_2, l_y_1, l_y_2, dt, n_max, n_min, alpha_max);
	initializeSensors();
	initializeMatrices();
	solver.initializeSolver(dt);
	wind.setData(wind_speed, wind_direction, dt);
	current.setData(current_speed, current_direction, dt);
	waves.setData(wave_height, wave_direction, dt, K_wave);
}

Vessel::~Vessel() {}

void Vessel::initializeStateVectors(){
	eta = Vector6d::Zero();
	nu  = Vector6d::Zero();
	nu_n = Vector6d::Zero();
	nu_r = Vector6d::Zero();
	nu_c_b = Vector6d::Zero();
	nu_c_n = Vector6d::Zero();
	tau_wind = Vector6d::Zero();
	tau_waves = Vector6d::Zero();
	tau_current = Vector6d::Zero();
	tau_control = Vector6d::Zero();
	tau_total = Vector6d::Zero();
	x_6_w =  Vector2d::Zero();

}

void Vessel::setGpsCoordinates(double _lat, double _long){
	gps.setCoordinates(_lat, _long);
}

void Vessel::initializeSensors(){
	gps.setStepSize(dt);
	gps.setCoordinates(start_latitude, start_longitude);
	gps.setFrequency(gps_frequency);
	speedSensor.setStepSize(dt);
	speedSensor.setFrequency(speed_sensor_frequency);
	windSensor.setStepSize(dt);
	windSensor.setFrequency(wind_sensor_frequency);
	mru.setStepSize(dt);
	mru.setFrequency(mru_frequency);
	imu.setStepSize(dt);
	imu.setFrequency(imu_frequency);
}

void Vessel::setThrust(Vector6d tau_in){
	tau_control = tau_in;
}

Vector6d Vessel::getThrust(){

	return tau_control;
}

void Vessel::setState(Vector6d eta_new, Vector6d nu_new){
	eta = eta_new;
	nu = nu_new;
}

void Vessel::getState(Vector6d &eta_2, Vector6d &nu_2){
	eta_2 = eta;
	nu_2 = nu;
}

void Vessel::initializeFluidMemoryMatrices(){
	mu << 0,0,0,0,0,0;
	A_11 << -10.5819,	-40.3442,	-74.3206,	-51.0862,
			1,			0, 			0, 			0,
			0, 			1, 			0, 			0,
			0,			0,			1,			0;

	A_22 << -10.8044, 	-40.2573, 	-74.2156, 	-51.4893,
			1,			0,			0,			0,
			0,			1,			0,			0,
			0,			0,			1,			0;

	A_33 << -5.9560,	-11.4690, 	-5.3749, 	-0.4502,
			1,			0,			0,			0,
			0,			1,			0,			0,
			0,			0,			1,			0;

	A_55 << -6.0199, 	-11.3013, 	-5.2349, 	-0.4339,
			1,			0,			0,			0,
			0,			1,			0,			0,
			0,			0,			1,			0;

	A_66 << -11.0792, 	-43.8660, 	-84.4465, 	-63.9643,
			1,			0,			0,			0,
			0,			1,			0,			0,	
			0,			0,			1,			0;

	A_53 << -6.4986, 	-31.5127, 	-16.5388, 	-1.4476,
			1,			0,			0,			0,
			0,			1,			0,			0,
			0,			0,			1,			0;

	A_35 = A_53;

	A_44 << -8.4332, 	-23.9192, 	-36.1083,
			1,			0,			0,
			0,			1,			0;

	A_24 << -8.1047, 	-23.4101, 	-34.5435,
			1,			0,			0,
			0,			1,			0;

	A_42 = A_24;

	A_26 << -1.1147,	-6.0631,
			1,			0;

	A_62 = A_26;

	A_46 << -1.5393, 	-5.8180,
			1,			0;

	A_64 = A_46;	

	B_11 << 1,
			0,
			0,
			0;

	B_22 << 1,
			0,
			0,
			0;

	B_33 << 1,
			0,
			0,
			0;

	B_55 << 1,
			0,
			0,
			0;

	B_66 << 1,
			0,
			0,
			0;

	B_53 << 1,
			0,
			0,
			0;
	
	B_35 << 1,
			0,
			0,
			0;		

	B_44 << 1,
			0,
			0;

	B_24 << 1,
			0,
			0;

	B_42 << 1,
			0,
			0;

	B_26 << 1,
			0;

	B_46 << 1,
			0;

	B_62 << 1,
			0;

	B_64 << 1,
			0;

	C_11 << 9979.32741355037, 22918.4583751628, 15780.8377225351, 0;

	C_22 << 40179.7498055168, 92673.0452638225, 64374.4305099363, 0;

	C_33 << 147159.294077335, 243503.238108501, 43814.5056222167, 0;

	C_55 << 434621.463430541, 665145.221962119, 116485.986335729, 0;

	C_66 << 123351.802797675, 300235.457336133, 227412.688726648, 0;

	C_53 << -42225.0493265813, 49487.0184208576, 14376.5689784988, 0;

	C_35 = C_53;

	C_44 << 28610.6674325093, 40464.3930955079, 0;

	C_24 << 30394.1087395157, 42384.5103537706, 0;

	C_42 = C_24;

	C_26 << -209.165956356670, 0;

	C_46 << -1263.19362192829, 0;

	C_62 = C_26;

	C_64 = C_46;

	x_11 << 0,
			0,
			0,
			0;

	x_22 = x_11;	
	x_33 = x_11;
	x_55 = x_11;
	x_66 = x_11;
	x_53 = x_11;
	x_35 = x_11;

	x_44 << 0,
			0,
			0;

	x_24 = x_44;
	x_42 = x_24;	

	x_26 << 0,
			0;
	x_46 = x_26;
	x_62 = x_26;
	x_64 = x_26;

}

void Vessel::calculateFluidMemoryEffects(){
	delta_nu = nu_r;
	delta_nu(0) = 0;
	x_11 = x_11 + dt*(A_11*x_11+B_11*delta_nu(0));
	
	x_22 = x_22 + dt*(A_22*x_22+B_22*delta_nu(1));
	x_24 = x_24 + dt*(A_24*x_24+B_24*delta_nu(3));
	x_26 = x_26 + dt*(A_26*x_26+B_26*delta_nu(5));
	
	x_33 = x_33 + dt*(A_33*x_33+B_33*delta_nu(2));
	x_35 = x_35 + dt*(A_35*x_35+B_35*delta_nu(4));
	
	x_42 = x_42 + dt*(A_42*x_42+B_42*delta_nu(1));
	x_44 = x_44 + dt*(A_44*x_44+B_44*delta_nu(3));
	x_46 = x_46 + dt*(A_46*x_46+B_46*delta_nu(5));
	
	x_53 = x_53 + dt*(A_53*x_53+B_53*delta_nu(2));
	x_55 = x_55 + dt*(A_55*x_55+B_55*delta_nu(4));
	
	x_62 = x_62 + dt*(A_62*x_62+B_62*delta_nu(1));
	x_64 = x_64 + dt*(A_64*x_64+B_64*delta_nu(3));
	x_66 = x_66 + dt*(A_66*x_66+B_66*delta_nu(5));

	double mu_1, mu_2, mu_3, mu_4, mu_5, mu_6;

	// The fluid memory effects model used in this prototype is calculated for the Viknes830-vessel. A simple transformation of the forces and momemnts using the BIS-system is performed here, 
	// to roughly "fit" the data to the vessel simulated. 

	mu_1 = double(C_11*x_11)*((m_11*L_pp)/(L_pp*9.81))/((7.2*5002.6)/(7.2*9.81));
	mu_2 = (double(C_22*x_22)+double(C_24*x_24)+double(C_26*x_26))*((m_11*L_pp)/(L_pp*9.81))/((7.2*5002.6)/(7.2*9.81));
	mu_3 = (double(C_33*x_33)+double(C_35*x_35))*((m_11*L_pp)/(L_pp*9.81))/((7.2*5002.6)/(7.2*9.81));
	mu_4 = (double(C_42*x_42)+double(C_44*x_44)+double(C_46*x_46))*((m_11*L_pp*L_pp)/(L_pp*9.81))/((7.2*7.2*5002.6)/(7.2*9.81));
	mu_5 = (double(C_53*x_53)+double(C_55*x_55))*((m_11*L_pp*L_pp)/(L_pp*9.81))/((7.2*7.2*5002.6)/(7.2*9.81));
	mu_6 = (double(C_62*x_62)+double(C_64*x_64)+double(C_66*x_66))*((m_11*L_pp*L_pp)/(L_pp*9.81))/((7.2*7.2*5002.6)/(7.2*9.81));

	mu << mu_1, mu_2, mu_3, mu_4, mu_5, mu_6;
}	

void Vessel::calculateNonlinearSurge(){
	double _u_r = nu_r(0);
	if(_u_r<0.2)
		_u_r=0.2;
	double R_n = (_u_r*L_pp)/(1.0*pow(10, -6));
	double C_F = pow((log10(R_n)-2),2);	
	// TODO: Update the nonlinear surge coefficient
	X_uu = X_uu_c*((0.075/C_F)+0.014); 
}
void Vessel::calculateCrossFlowDrag(){
	double rho = 1025;
	double _v = nu_r(1);
	double _r = nu_r(5);
	Y_vv = 0;
	N_vv = 0;
	double _dt=0.01;
	for(double x=(-L_pp)/2; x<=(0.9*L_pp)/2;){
		Y_vv+=_dt*(T*C_d_2d*std::abs(_v+x*_r)*(_v+x*_r));
		N_vv+=_dt*(T*C_d_2d*x*std::abs(_v+x*_r)*(_v+x*_r));
		x = x+_dt;
	}

	if(std::abs(_v)>=0.05){
		Y_vv = -(Y_vv*(rho/2))/(_v);
		N_vv = -(N_vv*(rho/2))/(_v);
	}
	
}

void Vessel::updateMatrices(){
	double u, v, w, p, q, r, phi, theta, psi;
	u = nu_r(0);
	v = nu_r(1);
	w = nu_r(2);
	p = nu_r(3);
	q = nu_r(4);
	r = nu_r(5);
	phi = eta(3);
	theta=eta(4);
	psi=eta(5);

	//Coriolis and centripetal

	//Linearized
	C_rb << 0,	0,	0,	0,	0,	0,
			0,	0,	0,	0,	0,	C_rb_26,
			0,	0,	0,	0,	C_rb_35,	0,
			0,	0,	0,	0,	0,	C_rb_46,
			0,	0,	0,	0,	0,	0,	
			0,	0,	0,	0,	0,	0;

	C_a <<  0,	0,	0,	0,	0,	0,	
			0,	0,	0,	0,	0,	C_a_26,
			0,	0,	0,	0,	C_a_35, 0,
			0,	0,	0,	0,	0,	C_a_46,
			0,	0,	0,	0,	C_a_55, 0,
			0,	0,	0,	0,	0,	C_a_66;
	C = u*(C_rb+C_a);
	/*
	// Complete
	
	double a_1, a_2, a_3, b_1, b_2, b_3;
	a_1 = -1*(X_u_dot*u+X_v_dot*v+X_w_dot*w+X_p_dot*p+X_q_dot*q+X_r_dot*r);
	a_2 = -1*(Y_u_dot*u+Y_v_dot*v+Y_w_dot*w+Y_p_dot*p+Y_q_dot*q+Y_r_dot*r);
	a_3 = -1*(Z_u_dot*u+Z_v_dot*v+Z_w_dot*w+Z_p_dot*p+Z_q_dot*q+Z_r_dot*r);
	b_1 = -1*(K_u_dot*u+K_v_dot*v+K_w_dot*w+K_p_dot*p+K_q_dot*q+K_r_dot*r);
	b_2 = -1*(M_u_dot*u+M_v_dot*v+M_w_dot*w+M_p_dot*p+M_q_dot*q+M_r_dot*r);
	b_3 = -1*(N_u_dot*u+N_v_dot*v+N_w_dot*w+N_p_dot*p+N_q_dot*q+N_r_dot*r);

	C_rb << 0, 							0,					0,						m_11*(y_g*q+z_g*r),		-m_11*(x_g*q-w),		-m_11*(x_g*r+v),
			0,							0,					0,						-m_11*(y_g*p+w),		m_11*(z_g*r+x_g*p),		-m_11*(y_g*r-u),
			0, 							0,					0,						-m_11*(z_g*p-v),		-m_11*(z_g*q+u), 		m_11*(x_g*p+y_g*q),
			-m_11*(y_g*q+z_g*r),		m_11*(y_g*p+w), 	m_11*(z_g*p-v),			0,						m_66*r,					-m_55*q,	
			m_11*(x_g*q-w),				-m_11*(z_g*r+x_g*p),m_11*(z_g*q+u),			-m_66*r,				0, 						m_44*p,	
			m_11*(x_g*r+v), 			m_11*(y_g*r-u), 	-m_11*(x_g*p+y_g*q),	m_55*q,					-m_44*p,				0;



	C_a << 	0, 		0, 		0, 		0, 		-a_3, 	a_2,
			0, 		0, 		0, 		a_3, 	0, 		-a_1,
			0, 		0, 		0, 		-a_2, 	a_1, 	0,
			0, 		-a_3, 	a_2, 	0, 		-b_3,	b_2,
			a_3, 	0, 		-a_1, 	b_3, 	0, 		-b_1,
			-a_2, 	a_1, 	0, 		-b_2, 	b_1, 	0;

	C = C_rb+C_a;*/

	// Damping matrix. Contains non-linear elements for the standard 3DOF representation, and linear elements in the restoring 3 DOFs.
	calculateNonlinearSurge();
	calculateCrossFlowDrag();
	//std::cout << "Y_vv: " << Y_vv << ", N_vv: " << N_vv << std::endl;
	D << 	-X_u-X_uu*std::abs(u)-X_uuu*u*u, 0, 										0, 										0, 		0, 		0, 
			0, 							-Y_v-Y_vv-Y_vvv*v*v-Y_rv*std::abs(r), 0, 										0, 		0, 		-Y_r-Y_vr*std::abs(v)-Y_rr*std::abs(r)-Y_rrr*r*r,
			0, 							0, 										-Z_w, 									0, 		-Z_q, 		0, 
			0, 							0, 										0, 										-K_p, 	0, 		-K_r, 
			0, 							0, 										0, 										0, 		-M_q - M_qq*std::abs(q),	-M_r, 
			0, 							-N_v-N_vv-N_rv*std::abs(r)-N_vvv*v*v, 0, 										0, 		0, 		-N_r-N_vr*std::abs(v)-N_rr*std::abs(r)-N_rrr*r*r;

	// Kinematics:
	J << 	cos(psi)*cos(theta), 	-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), 	sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta), 	0, 	0, 						0,
			sin(psi)*cos(theta), 	cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), 	-cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi), 	0, 	0, 						0,
			-sin(theta), 			cos(theta)*sin(phi), 								cos(theta)*cos(phi), 								0, 	0, 						0,
			0, 						0, 													0, 													1, 	sin(phi)*tan(theta), 	cos(phi)*tan(theta),
			0, 						0, 													0, 													0, 	cos(phi), 				-sin(theta),
			0, 						0, 													0, 													0, 	sin(phi)/cos(theta), 	cos(phi)/cos(theta);
}

void Vessel::step(){
	updateMatrices();
	calculateFluidMemoryEffects();
	calculateNextEta();
	calculateNextNu();
	publishSensorData();
	calculateEnvironmentalForces();
	//getWaveYawMoment();
	//std::cout << nu_r << std::endl;
}

void Vessel::calculateEnvironmentalForces(){
	calculateWindForces();
	calculateCurrentForces();
	if(wave_height!=0)
		waves.getForces(tau_waves, eta(5), nu(0));
}

void Vessel::calculateCurrentForces(){
	current.getData(current_speed, current_direction);
	current_direction = current_direction*(M_PI/180);
	nu_c_n << current_speed*cos(current_direction), current_speed*sin(current_direction), 0, 0, 0, 0;
	nu_c_b = J.transpose()*nu_c_n;
	nu_r = nu-nu_c_b;
}

void Vessel::getRelativeWindParameters(double &speed, double &direction){
	double u = nu(0);
	double v = nu(1);
	double psi = eta(5);
	double beta = direction*(M_PI/180);
	double u_w = speed*cos(beta-psi);
	double v_w = speed*sin(beta-psi);
	double u_rw = u-u_w;
	double v_rw = v-v_w;
	speed = sqrt(u_rw*u_rw+v_rw*v_rw);
	direction = -atan2(v_rw, u_rw);
}

void Vessel::calculateWindForces(){
	if(A_Lw == 0 || A_Fw == 0){
		tau_wind << 0,0,0,0,0,0;
	}else{
		wind.getData(wind_speed, wind_direction);
		getRelativeWindParameters(wind_speed, wind_direction);
		double gamma_w = wind_direction;
		while(std::abs(gamma_w)>M_PI){
			if(gamma_w>M_PI)
				gamma_w=gamma_w-2*M_PI;
			if(gamma_w<-M_PI)
				gamma_w=gamma_w+2*M_PI; 
		}
		double rho_a = 1.204;
		double q = 0.5*rho_a*wind_speed*wind_speed;
		double CD_l_af;
		if(std::abs(gamma_w)<M_PI/2)
			CD_l_af = CD_l_af_0;
		else
			CD_l_af = CD_l_af_pi;
		double CD_l = CD_l_af;
		double C_X = -CD_l*(A_Lw/A_Fw)*(cos(gamma_w)/(1-0.5*delta*(1-CD_l/CD_t)*sin(2*gamma_w)*sin(2*gamma_w)));
		double C_Y = CD_t*(sin(gamma_w)/(1-0.5*delta*(1-CD_l/CD_t)*sin(2*gamma_w)*sin(2*gamma_w)));
		double C_K = kappa*C_Y;
		double C_N;
		if(gamma_w<0)
			C_N = C_Y*((s_L/L_pp)-0.18*(-gamma_w-M_PI/2));
		else
			C_N = C_Y*((s_L/L_pp)-0.18*(gamma_w-M_PI/2));
		tau_wind << q*C_X*A_Fw, q*C_Y*A_Lw, 0, q*C_K*A_Lw*s_H, 0, q*C_N*A_Lw*L_pp;
	}
	
}

void Vessel::publishSensorData(){
	nu_n = (J*nu);
	gps.publishGpsData(nu_n, eta);
	imu.publishImuData(nu_dot , nu);
	mru.publishMruData(nu, eta);
	speedSensor.publishSpeedSensorData(nu(0), nu(1));
	windSensor.publishWindSensorData(wind_speed, wind_direction);
}

double Vessel::getDT(){
	return solver.h;
}

Vector6d Vessel::etaFunction(Vector6d nu_in){
    Vector6d eta_dot = J*nu_in;
   	return eta_dot;
} 

Vector6d Vessel::nuFunction(Vector6d nu_in){
	tau_total = tau_control+tau_wind+tau_waves;
	Vector6d nu_r_dot = -M_inv*(C*nu_in+D*nu_in+G*eta + mu -tau_total);
	return nu_r_dot;
}

void Vessel::calculateNextEta(){
	solver.k1v = solver.h*etaFunction(nu);
	solver.k2v = solver.h*etaFunction(nu + solver.a21*solver.k1v);
	solver.k3v = solver.h*etaFunction(nu + solver.a31*solver.k1v + solver.a32*solver.k2v);
	solver.k4v = solver.h*etaFunction(nu + solver.a41*solver.k1v + solver.a42*solver.k2v + solver.a43*solver.k3v);
	solver.k5v = solver.h*etaFunction(nu + solver.a51*solver.k1v + solver.a52*solver.k2v + solver.a53*solver.k3v + solver.a54*solver.k4v);
	solver.k6v = solver.h*etaFunction(nu + solver.a61*solver.k1v + solver.a62*solver.k2v + solver.a63*solver.k3v + solver.a64*solver.k4v + solver.a65*solver.k5v);
	solver.k7v = solver.h*etaFunction(nu + solver.a71*solver.k1v + solver.a73*solver.k3v + solver.a74*solver.k4v + solver.a75*solver.k5v + solver.a76*solver.k6v);
	eta = eta + solver.b11*solver.k1v + solver.b12*solver.k2v + solver.b13*solver.k3v + solver.b14*solver.k4v + solver.b15*solver.k5v + solver.b16*solver.k6v + solver.b17*solver.k7v;
}

void Vessel::calculateNextNu(){
	solver.k1v = solver.h*nuFunction(nu_r);
	solver.k2v = solver.h*nuFunction(nu_r + solver.a21*solver.k1v);
	solver.k3v = solver.h*nuFunction(nu_r + solver.a31*solver.k1v + solver.a32*solver.k2v);
	solver.k4v = solver.h*nuFunction(nu_r + solver.a41*solver.k1v + solver.a42*solver.k2v + solver.a43*solver.k3v);
	solver.k5v = solver.h*nuFunction(nu_r + solver.a51*solver.k1v + solver.a52*solver.k2v + solver.a53*solver.k3v + solver.a54*solver.k4v);
	solver.k6v = solver.h*nuFunction(nu_r + solver.a61*solver.k1v + solver.a62*solver.k2v + solver.a63*solver.k3v + solver.a64*solver.k4v + solver.a65*solver.k5v);
	solver.k7v = solver.h*nuFunction(nu_r + solver.a71*solver.k1v + solver.a73*solver.k3v + solver.a74*solver.k4v + solver.a75*solver.k5v + solver.a76*solver.k6v);
	nu_r = nu_r + solver.b11*solver.k1v + solver.b12*solver.k2v + solver.b13*solver.k3v + solver.b14*solver.k4v + solver.b15*solver.k5v + solver.b16*solver.k6v + solver.b17*solver.k7v;
	nu_dot = -M_inv*(C*nu+D*nu+G*eta-tau_total);
	nu = nu_r + nu_c_b;

}

void Vessel::initializeMatrices(){
	initializeFluidMemoryMatrices();
	M_det = 0;
	M_rb << m_11, m_12, m_13, m_14, m_15, m_16, 
          	m_21, m_22, m_23, m_24, m_25, m_26, 
			m_31, m_32, m_33, m_34, m_35, m_36, 
			m_41, m_42, m_43, m_44, m_45, m_46, 
			m_51, m_52, m_53, m_54, m_55, m_56, 
			m_61, m_62, m_63, m_64, m_65, m_66;

	M_a << 	X_u_dot, X_v_dot, X_w_dot, X_p_dot, X_q_dot, X_r_dot,
			Y_u_dot, Y_v_dot, Y_w_dot, Y_p_dot, Y_q_dot, Y_r_dot,
			Z_u_dot, Z_v_dot, Z_w_dot, Z_p_dot, Z_q_dot, Z_r_dot,
			K_u_dot, K_v_dot, K_w_dot, K_p_dot, K_q_dot, K_r_dot,
			M_u_dot, M_v_dot, M_w_dot, M_p_dot, M_q_dot, M_r_dot,
			N_u_dot, N_v_dot, N_w_dot, N_p_dot, N_q_dot, N_r_dot;
	M = M_rb+M_a;

	G << 	0, 	0,	0,		0,		0,			0,	
			0,	0,	0,		0,		0,			0,
			0,	0,	-Z_z,	0,		-Z_theta, 	0,
			0,	0,	0,		-K_phi, 0,			0,	
			0,	0,	-M_z, 	0, 		-M_theta, 	0,
			0,	0,	0,		0,		0,			0;

	M_det = M.determinant();
	M_inv = M.inverse();
}

bool Vessel::readParameters(ros::NodeHandle nh) {
	bool parameterFail = false;

	// Mass matrix M_rb
	if (!nh.getParam("m_11", m_11))
		parameterFail=true;
	if (!nh.getParam("m_12", m_12))
		parameterFail=true;
	if (!nh.getParam("m_13", m_13))
		parameterFail=true;
	if (!nh.getParam("m_14", m_14))
		parameterFail=true;
	if (!nh.getParam("m_15", m_15))
		parameterFail=true;
	if (!nh.getParam("m_16", m_16))
		parameterFail=true;

	if (!nh.getParam("m_21", m_21))
		parameterFail=true;
	if (!nh.getParam("m_22", m_22))
		parameterFail=true;
	if (!nh.getParam("m_23", m_23))
		parameterFail=true;
	if (!nh.getParam("m_24", m_24))
		parameterFail=true;
	if (!nh.getParam("m_25", m_25))
		parameterFail=true;
	if (!nh.getParam("m_26", m_26))
		parameterFail=true;

	if (!nh.getParam("m_31", m_31))
		parameterFail=true;
	if (!nh.getParam("m_32", m_32))
		parameterFail=true;
	if (!nh.getParam("m_33", m_33))
		parameterFail=true;
	if (!nh.getParam("m_34", m_34))
		parameterFail=true;
	if (!nh.getParam("m_35", m_35))
		parameterFail=true;
	if (!nh.getParam("m_36", m_36))
		parameterFail=true;

	if (!nh.getParam("m_41", m_41))
		parameterFail=true;
	if (!nh.getParam("m_42", m_42))
		parameterFail=true;
	if (!nh.getParam("m_43", m_43))
		parameterFail=true;
	if (!nh.getParam("m_44", m_44))
		parameterFail=true;
	if (!nh.getParam("m_45", m_45))
		parameterFail=true;
	if (!nh.getParam("m_46", m_46))
		parameterFail=true;

	if (!nh.getParam("m_51", m_51))
		parameterFail=true;
	if (!nh.getParam("m_52", m_52))
		parameterFail=true;
	if (!nh.getParam("m_53", m_53))
		parameterFail=true;
	if (!nh.getParam("m_54", m_54))
		parameterFail=true;
	if (!nh.getParam("m_55", m_55))
		parameterFail=true;
	if (!nh.getParam("m_56", m_56))
		parameterFail=true;

	if (!nh.getParam("m_61", m_61))
		parameterFail=true;
	if (!nh.getParam("m_62", m_62))
		parameterFail=true;
	if (!nh.getParam("m_63", m_63))
		parameterFail=true;
	if (!nh.getParam("m_64", m_64))
		parameterFail=true;
	if (!nh.getParam("m_65", m_65))
		parameterFail=true;
	if (!nh.getParam("m_66", m_66))
		parameterFail=true;

	// Added mass matrix M_a
	if (!nh.getParam("X_u_dot", X_u_dot))
		parameterFail=true;
	if (!nh.getParam("X_v_dot", X_v_dot))
		parameterFail=true;
	if (!nh.getParam("X_w_dot", X_w_dot))
		parameterFail=true;
	if (!nh.getParam("X_p_dot", X_p_dot))
		parameterFail=true;
	if (!nh.getParam("X_q_dot", X_q_dot))
		parameterFail=true;
	if (!nh.getParam("X_r_dot", X_r_dot))
		parameterFail=true;

	if (!nh.getParam("Y_u_dot", Y_u_dot))
		parameterFail=true;
	if (!nh.getParam("Y_v_dot", Y_v_dot))
		parameterFail=true;
	if (!nh.getParam("Y_w_dot", Y_w_dot))
		parameterFail=true;
	if (!nh.getParam("Y_p_dot", Y_p_dot))
		parameterFail=true;
	if (!nh.getParam("Y_q_dot", Y_q_dot))
		parameterFail=true;
	if (!nh.getParam("Y_r_dot", Y_r_dot))
		parameterFail=true;

	if (!nh.getParam("Z_u_dot", Z_u_dot))
		parameterFail=true;
	if (!nh.getParam("Z_v_dot", Z_v_dot))
		parameterFail=true;
	if (!nh.getParam("Z_w_dot", Z_w_dot))
		parameterFail=true;
	if (!nh.getParam("Z_p_dot", Z_p_dot))
		parameterFail=true;
	if (!nh.getParam("Z_q_dot", Z_q_dot))
		parameterFail=true;
	if (!nh.getParam("Z_r_dot", Z_r_dot))
		parameterFail=true;

	if (!nh.getParam("K_u_dot", K_u_dot))
		parameterFail=true;
	if (!nh.getParam("K_v_dot", K_v_dot))
		parameterFail=true;
	if (!nh.getParam("K_w_dot", K_w_dot))
		parameterFail=true;
	if (!nh.getParam("K_p_dot", K_p_dot))
		parameterFail=true;
	if (!nh.getParam("K_q_dot", K_q_dot))
		parameterFail=true;
	if (!nh.getParam("K_r_dot", K_r_dot))
		parameterFail=true;

	if (!nh.getParam("M_u_dot", M_u_dot))
		parameterFail=true;
	if (!nh.getParam("M_v_dot", M_v_dot))
		parameterFail=true;
	if (!nh.getParam("M_w_dot", M_w_dot))
		parameterFail=true;
	if (!nh.getParam("M_p_dot", M_p_dot))
		parameterFail=true;
	if (!nh.getParam("M_q_dot", M_q_dot))
		parameterFail=true;
	if (!nh.getParam("M_r_dot", M_r_dot))
		parameterFail=true;

	if (!nh.getParam("N_u_dot", N_u_dot))
		parameterFail=true;
	if (!nh.getParam("N_v_dot", N_v_dot))
		parameterFail=true;
	if (!nh.getParam("N_w_dot", N_w_dot))
		parameterFail=true;
	if (!nh.getParam("N_p_dot", N_p_dot))
		parameterFail=true;
	if (!nh.getParam("N_q_dot", N_q_dot))
		parameterFail=true;
	if (!nh.getParam("N_r_dot", N_r_dot))
		parameterFail=true;

	//Center of gravity
	if (!nh.getParam("x_g", x_g))
		parameterFail=true;
	if (!nh.getParam("y_g", y_g))
		parameterFail=true;

	// Linear damping
	if (!nh.getParam("X_u", X_u))
		parameterFail=true;
	if (!nh.getParam("X_v", X_v))
		parameterFail=true;
	if (!nh.getParam("Y_v", Y_v))
		parameterFail=true;
	if (!nh.getParam("Y_r", Y_r))
		parameterFail=true;
	if (!nh.getParam("N_v", N_v))
		parameterFail=true;
	if (!nh.getParam("N_r", N_r))
		parameterFail=true;
	if (!nh.getParam("K_p", K_p))
		parameterFail=true;
	if (!nh.getParam("M_q", M_q))
		parameterFail=true;
	if (!nh.getParam("M_r", M_r))
		parameterFail=true;
	if (!nh.getParam("K_r", K_r))
		parameterFail=true;
	if (!nh.getParam("Z_q", Z_q))
		parameterFail=true;
	if (!nh.getParam("Z_w", Z_w))
		parameterFail=true;

	// Restoring forces
	if (!nh.getParam("Z_z", Z_z))
		parameterFail=true;
	if (!nh.getParam("Z_theta", Z_theta))
		parameterFail=true;
	if (!nh.getParam("K_phi", K_phi))
		parameterFail=true;
	if (!nh.getParam("M_z", M_z))
		parameterFail=true;
	if (!nh.getParam("M_theta", M_theta))
		parameterFail=true;

	// Nonlinear damping
	if (!nh.getParam("X_uu", X_uu))
		parameterFail=true;
	if (!nh.getParam("X_uuu", X_uuu))
		parameterFail=true;
	if (!nh.getParam("X_vv", X_vv))
		parameterFail=true;
	if (!nh.getParam("X_vvv", X_vvv))
		parameterFail=true;
	if (!nh.getParam("Y_vv", Y_vv))
		parameterFail=true;
	if (!nh.getParam("Y_vvv", Y_vvv))
		parameterFail=true;
	if (!nh.getParam("Y_rr", Y_rr))
		parameterFail=true;
	if (!nh.getParam("Y_rrr", Y_rrr))
		parameterFail=true;
	if (!nh.getParam("Y_rv", Y_rv))
		parameterFail=true;
	if (!nh.getParam("Y_vr", Y_vr))
		parameterFail=true;
	if (!nh.getParam("N_rr", N_rr))
		parameterFail=true;
	if (!nh.getParam("N_rrr", N_rrr))
		parameterFail=true;
	if (!nh.getParam("N_vv", N_vv))
		parameterFail=true;
	if (!nh.getParam("N_vvv", N_vvv))
		parameterFail=true;
	if (!nh.getParam("N_rv", N_rv))
		parameterFail=true;
	if (!nh.getParam("N_vr", N_vr))
		parameterFail=true;
	if (!nh.getParam("M_qq", M_qq))
		parameterFail=true;


	// Linearized Coriolis parameters
	if (!nh.getParam("C_rb_26", C_rb_26))
		parameterFail=true;
	if (!nh.getParam("C_rb_35", C_rb_35))
		parameterFail=true;
	if (!nh.getParam("C_rb_46", C_rb_46))
		parameterFail=true;

	if (!nh.getParam("C_a_26", C_a_26))
		parameterFail=true;
	if (!nh.getParam("C_a_35", C_a_35))
		parameterFail=true;
	if (!nh.getParam("C_a_46", C_a_46))
		parameterFail=true;
	if (!nh.getParam("C_a_55", C_a_55))
		parameterFail=true;
	if (!nh.getParam("C_a_66", C_a_66))
		parameterFail=true;

	//Wind parameters
	if (!nh.getParam("wind_speed", wind_speed))
		parameterFail=true;
	if (!nh.getParam("wind_direction", wind_direction))
		parameterFail=true;
	if (!nh.getParam("current_speed", current_speed))
		parameterFail=true;
	if (!nh.getParam("current_direction", current_direction))
		parameterFail=true;
	if (!nh.getParam("A_Fw", A_Fw))
		parameterFail=true;
	if (!nh.getParam("A_Lw", A_Lw))
		parameterFail=true;
	if (!nh.getParam("s_H", s_H))
		parameterFail=true;
	if (!nh.getParam("s_L", s_L))
		parameterFail=true;
	if (!nh.getParam("CD_t", CD_t))
		parameterFail=true;
	if (!nh.getParam("CD_l_af_0", CD_l_af_0))
		parameterFail=true;
	if (!nh.getParam("CD_l_af_pi", CD_l_af_pi))
		parameterFail=true;
	if (!nh.getParam("delta", delta))
		parameterFail=true;
	if (!nh.getParam("kappa", kappa))
		parameterFail=true;

	//Wave parameters
	if (!nh.getParam("wave_height", wave_height))
		parameterFail=true;
	if (!nh.getParam("wave_direction", wave_direction))
		parameterFail=true;
	if (!nh.getParam("K_1_w", K_1_w))
		parameterFail=true;
	if (!nh.getParam("K_2_w", K_2_w))
		parameterFail=true;
	if (!nh.getParam("K_3_w", K_3_w))
		parameterFail=true;
	if (!nh.getParam("K_4_w", K_4_w))
		parameterFail=true;
	if (!nh.getParam("K_5_w", K_5_w))
		parameterFail=true;
	if (!nh.getParam("K_6_w", K_6_w))
		parameterFail=true;

	// Thruster parameters
	if (!nh.getParam("K_thruster", K_thruster))
		parameterFail=true;
	if (!nh.getParam("l_y_1", l_y_1))
		parameterFail=true;
	if (!nh.getParam("l_y_2", l_y_2))
		parameterFail=true;
	if (!nh.getParam("l_x_1", l_x_1))
		parameterFail=true;
	if (!nh.getParam("l_x_2", l_x_2))
		parameterFail=true;
	if (!nh.getParam("T_n", T_n))
		parameterFail=true;
	if (!nh.getParam("T_alpha", T_alpha))
		parameterFail=true;
	if (!nh.getParam("T_beta", T_beta))
		parameterFail=true;
	if (!nh.getParam("n_min", n_min))
		parameterFail=true;
	if (!nh.getParam("n_max", n_max))
		parameterFail=true;
	if (!nh.getParam("alpha_max", alpha_max))
		parameterFail=true;

	// Sensor parameters
	if (!nh.getParam("gps_frequency", gps_frequency))
		parameterFail=true;
	if (!nh.getParam("mru_frequency", mru_frequency))
		parameterFail=true;
	if (!nh.getParam("imu_frequency", imu_frequency))
		parameterFail=true;
	if (!nh.getParam("speed_sensor_frequency", speed_sensor_frequency))
		parameterFail=true;
	if (!nh.getParam("wind_sensor_frequency", wind_sensor_frequency))
		parameterFail=true;
	if (!nh.getParam("start_latitude", start_latitude))
		parameterFail=true;
	if (!nh.getParam("start_longitude", start_longitude))
		parameterFail=true;
	if (!nh.getParam("surge_max", surge_max))
		parameterFail=true;

	//Others
	if (!nh.getParam("L_pp", L_pp))
		parameterFail=true;
	if (!nh.getParam("C_d_2d", C_d_2d))
		parameterFail=true;
	if (!nh.getParam("T", T))
		parameterFail=true;
	if (!nh.getParam("X_uu_c", X_uu_c))
		parameterFail=true;
	if (!nh.getParam("dt", dt))
		parameterFail=true;


	
	return parameterFail;
}
