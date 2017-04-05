#include "weather.h"

Weather::Weather() {}

Weather::~Weather() {}


void Weather::setData(double V_in, double beta_in, double _dt){
	mean_beta = beta_in;
	mean_V = V_in;
	beta = 0;
	V = 0;
	dt = _dt;
}

double Weather::generateRandomSpeed(){
	std::random_device rd;
	std::mt19937 gen(rd());
	double stdDev = 1;
	static std::normal_distribution<> d(0, stdDev);
	double res = d(gen);
	return res;
}

double Weather::generateRandomAngle(){
	std::random_device rd;
	std::mt19937 gen(rd());
	double stdDev = 1;
	static std::normal_distribution<> d(0, stdDev);
	double res = d(gen);
	return res;
}

double Weather::maintainSaturation(double current_value, double mean_value, double max_deviation){
	if(current_value>mean_value+max_deviation){
		current_value = mean_value+max_deviation;
	}else if(current_value<mean_value-max_deviation){
		current_value = mean_value-max_deviation;
	}
	return current_value;
}

void Weather::updateSpeed(){
	double w = generateRandomSpeed();
	double V_dot = w-mu_V*V;
	V = V+dt*V_dot;
}

void Weather::updateDirection(){
	double w = generateRandomAngle();
	double beta_dot = w-mu_beta*beta;
	beta = beta+dt*beta_dot;
}