#ifndef WAVES_H
#define WAVES_H

#include "weather.h"

class Waves : public Weather{
public: 
	Waves();
	~Waves();
	void setData(double H_s_in, double beta_in, double _dt, Vector6d K_wave_in);
	void getForces(Vector6d &tau_waves_in, double psi, double U);

private:
	double getForceOneDOF(Vector2d B_i, Vector2d &x_i, double omega_rand);
	void updateEncounterFrequency(double U, double psi);
	double H_s;
	double lambda = 0.1017;
	double omega_0, omega_e;
	Vector6d tau_waves, K_wave;
	double F_X, F_Y, F_Z, F_K, F_N, F_M;
	Vector2d B_1, B_2, B_3, B_4, B_5, B_6;
	Vector2d x_1, x_2, x_3, x_4, x_5, x_6;
	Matrix2d A;
};


#endif