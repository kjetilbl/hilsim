#include "wind.h"

Wind::Wind(){	
	mu_V = 0.1;
	mu_beta = 0.1;
	max_beta_deviation = 10;
	max_V_deviation = 3;
}
Wind::~Wind(){

}
double Wind::calculateWindSpeedAtHeight(double U_10, double z) {
	double e = 2.718281;
	double kappa = 0.0025;
  	return (U_10 * (5 / 2) * sqrt(kappa) *
                    std::log(z / (10 * pow(e, -2 / (5 * sqrt(kappa))))));
}

double Wind::generateRandomGust(){
	std::random_device rd;
	std::mt19937 gen(rd());
	double stdDev = 5;
	static std::normal_distribution<> d(0, stdDev);
	double res = d(gen);
	return res;
}

void Wind::updateGustSpeed(){
	double w = generateRandomGust();
	double V_gust_dot = w;
	V_gust = V_gust+dt*V_gust_dot;
}

void Wind::getData(double &V_in, double &beta_in){
	updateSpeed();
	updateDirection();
	updateGustSpeed();
	V_gust = maintainSaturation(V_gust, 0, max_V_deviation);
	V = maintainSaturation(V, 0, max_V_deviation);
	double V_total = V+V_gust+mean_V;
	
	V_in = calculateWindSpeedAtHeight(V_total, 1);
	beta = maintainSaturation(beta, 0, max_beta_deviation);
	beta_in = beta+mean_beta;
	geometry_msgs::Twist wind_msg;
	wind_msg.linear.x = V_gust;
	wind_msg.linear.y = V_total;
	wind_msg.linear.z = V;
	wind_msg.angular.z = beta;

	wind_pub.publish(wind_msg);
}