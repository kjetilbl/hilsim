#include "waves.h"

Waves::Waves(){

}
Waves::~Waves(){

}

void Waves::setData(double H_s_in, double beta_in, double _dt, Vector6d K_wave_in){
	H_s = H_s_in;
	if(H_s != 0){
		dt = _dt;
		omega_0 = pow(((3.11L/(H_s*H_s))/5),1.0L/4.0L);
		K_wave = K_wave_in;
		beta = beta_in;
		A << 	0,		1,
				-omega_0*omega_0, 	-2*lambda*omega_0;
		B_1 << 0, K_wave(0);
		B_2 << 0, K_wave(1);
		B_3 << 0, K_wave(2);
		B_4 << 0, K_wave(3);
		B_5 << 0, K_wave(4);
		B_6 << 0, K_wave(5);
	}
	
}

void Waves::updateEncounterFrequency(double U, double psi){
	double g = 9.81;
	omega_e = omega_0-((omega_0*omega_0)/g)*U*cos(beta*(M_PI/180)-psi);
	if(omega_e<0)
		omega_e = -omega_e;
	A << 	0, 		1,
			-omega_e*omega_e, -2*lambda*omega_e;

}

void Waves::getForces(Vector6d &tau_waves_in, double psi, double U){
	updateEncounterFrequency(U, psi);
	std::random_device rd;
	std::mt19937 gen(rd());
	double stdDev = H_s;
	static std::normal_distribution<> d(0, stdDev);
	double omega_rand = d(gen);
	double gamma = beta*(M_PI/180)-psi;
	F_X = getForceOneDOF(B_1, x_1, omega_rand);
	F_Y = getForceOneDOF(B_2, x_2, omega_rand);
	F_Z = getForceOneDOF(B_3, x_3, omega_rand);
	F_K = getForceOneDOF(B_4, x_4, omega_rand);
	F_M = getForceOneDOF(B_5, x_5, omega_rand);
	F_N = getForceOneDOF(B_6, x_6, omega_rand);
	tau_waves_in << F_X*-std::abs(cos(gamma)), F_Y*-std::abs(sin(gamma)), F_Z, F_K*sin(gamma), F_N*cos(gamma), F_M;
}

double Waves::getForceOneDOF(Vector2d B_i, Vector2d &x_i, double omega_rand){
	x_i = x_i+dt*(A*x_i+B_i*omega_rand);
	return x_i(1);
}