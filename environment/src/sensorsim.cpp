
#include "sensorsim.h"
#include "simulator_messages/Gps.h"
#include <QTime>
#include "stdint.h"
#include <math.h>
#include <random>

#define ASCII_CHAR_OFFSET 	48
#define SIX_BIT_MASK 		0b00111111
#define FIVE_BIT_MASK 		0b00011111
#define FOUR_BIT_MASK 		0b00001111
#define THREE_BIT_MASK 		0b00000111
#define TWO_BIT_MASK 		0b00000011
#define ONE_BIT_MASK 		0b00000001

uint32_t detectedObject::targetIterator = 0;

//----------------------------------------------------------------------------------------
//----------------------------------------sensorSim----------------------------------------
//----------------------------------------------------------------------------------------


sensorSim::sensorSim(ros::NodeHandle *n, QThread *parent) : QThread(parent)
{
	nh = n;
	if( !read_sensor_config() ){
		qDebug() << "sensorSim read sensor config failed.";
		exit(0);
	}
}

sensorSim::~sensorSim()
{
	delete AIStimer;
	delete DTtimer;
}

void sensorSim::run()
{
	USVnavData = navData(316001245, 123.8777500, 49.2002817, 19.6, 235.0);
	ros::Subscriber USVupdateSub = nh->subscribe("sensors/gps", 1000, &sensorSim::USV_gps_parser, this);
	ros::Subscriber obstUpdateSub = nh->subscribe("/simObject/position", 1000, &sensorSim::obstacle_update_parser, this);
	ros::Subscriber AISsub = nh->subscribe("sensors/ais", 1000, &sensorSim::AIS_parser, this);
	detectedTargetPub = nh->advertise<simulator_messages::detectedTarget>("sensors/target_detection", 1000);

	AIStimer = new QTimer();
	DTtimer = new QTimer();
	QObject::connect( AIStimer, SIGNAL(timeout()), this, SLOT(print_USV_AIS_msg()) );
	QObject::connect( DTtimer, SIGNAL(timeout()), this, SLOT(publish_detected_targets()) );
	AIStimer->start(2000);
	DTtimer->start(100);

	QThread::exec();
}

bool sensorSim::read_sensor_config(){
	bool successfulRead = true;
	if(!nh->getParam("radar_range", radarRange)){
		radarRange = 200;
		successfulRead = false;
	}
	return successfulRead;
}

void sensorSim::print_USV_AIS_msg()
{
	std::lock_guard<std::mutex> lock(m);
	USVnavData.set_time(QTime::currentTime());
}


void sensorSim::publish_detected_targets()
{
	std::lock_guard<std::mutex> lock(m);

	vector<string> outdatedObstacles;

	// Publish obstacle position info and find outdated obstacles
	for( auto & obst : unidentifiedObjects )
	{
		string obstID = obst.first;
		if( !obst.second.AIS_ON && !obst.second.radar_ON )
		{
			outdatedObstacles.push_back(obstID);
		}
		else
		{
			double distanceToObject = distance_m(USVnavData.get_position(), obst.second.get_true_position());
			obst.second.estimate_states(distanceToObject);
			simulator_messages::detectedTarget dt = obst.second.makeDTmsg();

			detectedTargetPub.publish(dt);
		}
	}
	// Delete old obstacles
	for ( auto const& ID : outdatedObstacles ) 
	{
    	unidentifiedObjects.erase(ID);
	}
}


void sensorSim::USV_gps_parser(const simulator_messages::Gps::ConstPtr& USVgpsMsg)
{
	std::lock_guard<std::mutex> lock(m);
	USVnavData.set_position_accuracy(HIGH);
	USVnavData.set_nav_status(UNDERWAY_USING_ENGINE);
	USVnavData.set_position(USVgpsMsg->longitude, USVgpsMsg->latitude);
	USVnavData.set_track(USVgpsMsg->heading);
	USVnavData.set_COG(USVgpsMsg->heading);
	USVnavData.set_ROT(USVgpsMsg->headingRate);
	USVnavData.set_SOG(USVgpsMsg->speed);
	USVnavData.set_time(QTime::currentTime());
}

void sensorSim::obstacle_update_parser(const environment::obstacleUpdate::ConstPtr& obstUpdateMsg)
{
	std::lock_guard<std::mutex> lock(m);

	string objectDescriptor = obstUpdateMsg->objectDescriptor;
	gpsPoint obstPos(obstUpdateMsg->longitude, obstUpdateMsg->latitude);
	double crossSection = obstUpdateMsg->size*6;
	double COG = obstUpdateMsg->heading;
	string objectID = obstUpdateMsg->objectID;

	if( !is_within_visibility(obstPos, crossSection, objectID) ){
		return;
	}
	map<string, detectedObject>::iterator it = unidentifiedObjects.find(objectID);
	if( it != unidentifiedObjects.end() ) // Object already detected
	{
		unidentifiedObjects[objectID].set_radar_data(obstPos, crossSection);
	}else
	{
		if (objectDescriptor == "fixed_obstacle")
		{
			unidentifiedObjects[objectID] = detectedObject(*nh, objectDescriptor, obstPos, 0, 0, crossSection);
		}
		else if (objectDescriptor == "ship")
		{
			unidentifiedObjects[objectID] = detectedObject(*nh, "vessel", obstPos, 0, 0, crossSection);
		}
	}
	unidentifiedObjects[objectID].radar_ON = true;
}

void sensorSim::AIS_parser(const simulator_messages::AIS::ConstPtr& AISmsg)
{

	std::lock_guard<std::mutex> lock(m);
	navData nd(AISmsg->MMSI, AISmsg->longitude, AISmsg->latitude, AISmsg->SOG, AISmsg->track);
	nd.set_nav_status((navStatus)AISmsg->status);
	nd.set_ROT(AISmsg->ROT);
	nd.set_position_accuracy((posAccuracy)AISmsg->positionAccuracy);
	nd.set_COG(AISmsg->COG);
	nd.set_time(QTime(AISmsg->hour, AISmsg->minute, AISmsg->second));
	detectedAISusers[AISmsg->MMSI] = nd;

	string objectDescriptor = "vessel";
	string objectID = "AIS_user_" + to_string(AISmsg->MMSI);
	gpsPoint position(AISmsg->longitude, AISmsg->latitude);

	double COG = AISmsg->COG;
	double SOG = AISmsg->SOG;
	double ROT = AISmsg->ROT;
	double crossSection = pow(150,2);
	map<string, detectedObject>::iterator it = unidentifiedObjects.find(objectID);
	if( it != unidentifiedObjects.end() ) // Object already detected
	{
		unidentifiedObjects[objectID].set_AIS_data(SOG, COG, ROT, position);
	}else
	{
		unidentifiedObjects[objectID] = detectedObject(*nh, objectDescriptor, position, COG, SOG, crossSection);
	}
	unidentifiedObjects[objectID].AIS_ON = true;
}

bool sensorSim::is_within_visibility(gpsPoint newPos, double crossSection, string objectID = "unknown"){
	//Check range
	gpsPointStamped usvPos = USVnavData.get_position();
	double distToNewPos = distance_m(usvPos, newPos);
	if (distToNewPos > radarRange)
	{
		return false;
	}
	if (sqrt(crossSection)/distToNewPos < 0.01)
	{
		return false;
	}

	// Check if behind other objects
	for( auto & mapPair : unidentifiedObjects ){
		gpsPoint objectPos = mapPair.second.get_true_position();
		double distToObject = distance_m(usvPos, objectPos);
		double objectRadius = sqrt(mapPair.second.get_true_CS())/2;
		if( distToNewPos < distToObject ) 
		{
			continue;
		}
		if( mapPair.first == objectID ){
			// No need to check if the object is behind itself.
			continue;
		}
		double newPosBearing = compass_bearing(usvPos, newPos);
		double maxBearing = compass_bearing(usvPos, objectPos) + atan2(objectRadius, distToObject)*180/M_PI;
		double minBearing = compass_bearing(usvPos, objectPos) - atan2(objectRadius, distToObject)*180/M_PI;
		if ( is_within_bearing_range(newPosBearing, minBearing, maxBearing) )
		{
			return false;
		}
	}

	//
	return true;
}

detectedObject::detectedObject(){}

detectedObject::detectedObject(	ros::NodeHandle nh,
								string objectDescriptor,
								gpsPoint truePosition, 
								double trueCOG, 
								double trueSOG, 
								double trueCrossSection)
{
	descriptor = objectDescriptor;
	targetNumber = targetIterator++;
	X = Eigen::VectorXd(n);
	Xe = Eigen::VectorXd(n);
	br = Eigen::VectorXd::Zero(nr);
	Tdr = Eigen::MatrixXd::Zero(nr, nr);

	Za = Eigen::VectorXd(na);
	Za << trueSOG, trueCOG, 0, truePosition.longitude, truePosition.latitude;
	Za_prev = Eigen::VectorXd::Zero(na);
	Xr_bar = Eigen::VectorXd(nk);
	Xr_bar << Za, Eigen::VectorXd::Zero(7);
	Xr_hat = Xr_bar;
	Xa_bar = Xr_bar;
	Xa_hat = Xr_bar;
	Pa_bar = 10*Eigen::MatrixXd::Identity(nk,nk);
	Pr_bar = 100*Eigen::MatrixXd::Identity(nk,nk);
	Pr_bar(0,0) = 1000000;
	Pr_bar(1,1) = 1000000;
	Pr_bar(2,2) = 1000000;
	double u = Xa_hat(0);
	double psi = deg2rad( Xa_hat(1) );
	double rot = deg2rad( Xa_hat(2) );
	fa_hat = Eigen::VectorXd(nk);
	fa_hat << 0, rot, 0, u*sin(psi)*longitude_degs_pr_meter(truePosition.latitude), u*cos(psi)*latitude_degs_pr_meter(), 0, 0, 0, 0, 0, 0, 0;
	cout << Xa_hat;
	if(!read_sensor_config(nh, truePosition)){
		qDebug() << "detectedObject read sensor config failed.";
		exit(-1);
	}

	update_true_states(truePosition, trueCOG, trueSOG, trueCrossSection);
	lastAISupdate = QTime::currentTime();
	lastRADARupdate = QTime::currentTime();
}

Eigen::VectorXd detectedObject::estimate_states( double distanceFromUSV_m ){
	QTime now = QTime::currentTime();
	if (lastAISupdate.msecsTo(now) > 5000)
	{
		AIS_ON = false;
	}
	if (lastRADARupdate.msecsTo(now) > 2000)
	{
		radar_ON = false;
	}

	if( radar_ON ){
		Zr = generate_radar_measurement(distanceFromUSV_m);
	}
	Xe = KalmanFusion();
	return Xe;
}

Eigen::VectorXd detectedObject::generate_radar_measurement( double distanceFromUSV_m ){
	static std::default_random_engine randomGenerator;
	static std::normal_distribution<double> gaussianWhiteNoise(0,1);

	if (firstTimeParamEstimate)
	{
		for (int i = 0; i < nr; i++)
		{
			br(i) = gaussianWhiteNoise(randomGenerator)*brSigmas(i);
		}
		firstTimeParamEstimate = false;
		lastEstimateTime = QTime::currentTime();
	}
	
	QTime now = QTime::currentTime();
	double dt = (double)(lastEstimateTime.msecsTo( now ))/1000;

	Eigen::VectorXd e(nr);
	Eigen::VectorXd w(nr);
	Eigen::VectorXd v(nr);
	for (int i = 0; i < nr; i++)
	{
		Tdr(i,i) = exp(-1/Tr(i,i)*dt);
		w(i) = gaussianWhiteNoise(randomGenerator)*brSigmas(i);
		v(i) = gaussianWhiteNoise(randomGenerator)*ZrSigmas(i);
	}

	// Make artificial deviations
	br = Tdr*br + w;
	e = br + v;
	// e = errorPrDistanceGain * distanceFromUSV_m * e;
	e(2) = max(-X(2) + 1, (double)e(2));


	// Make artificial measurements:
	Eigen::VectorXd Z(nr);
	Eigen::VectorXd trueStates(nr);
	trueStates << X(0), X(1), X(4);
	Z = trueStates + e;

	lastEstimateTime = now;

	return Z;
}

void detectedObject::update_true_states(gpsPoint pos, double COG, double SOG, double crossSection){
	set_true_position(pos);
	set_true_COG(COG);
	set_true_SOG(SOG);
	set_true_cross_section(crossSection);
}

simulator_messages::detectedTarget detectedObject::makeDTmsg(){
	simulator_messages::detectedTarget dt;
	dt.targetID = get_target_number();
	dt.objectDescriptor = descriptor;
	dt.longitude = get_estimated_position().longitude;
	dt.latitude = get_estimated_position().latitude;
	dt.COG = round( get_estimated_COG() );
	dt.SOG = round( get_estimated_SOG() );
	dt.crossSection = get_estimated_CS();
	return dt;
}

bool detectedObject::read_sensor_config(ros::NodeHandle nh, gpsPoint truePosition){
	bool successfulRead = true;
	brSigmas = Eigen::VectorXd(nr);
	double positionBiasSigma = 0;
	if(!nh.getParam("radar_position_bias_sigma", positionBiasSigma)){
		successfulRead = false;
	}
	brSigmas(0) = positionBiasSigma*longitude_degs_pr_meter(truePosition.latitude);
	brSigmas(1) = positionBiasSigma*latitude_degs_pr_meter();


	if(!nh.getParam("radar_size_bias_sigma", brSigmas(2))){
		successfulRead = false;
		brSigmas(2) = 0;
	}

	ZrSigmas = Eigen::VectorXd(nr);
	double positionMeasureSigma = 0;
	if(!nh.getParam("radar_position_measure_sigma", positionMeasureSigma)){
		successfulRead = false;
	}
	ZrSigmas(0) = positionMeasureSigma*longitude_degs_pr_meter(truePosition.latitude);
	ZrSigmas(1) = positionMeasureSigma*latitude_degs_pr_meter();

	if(!nh.getParam("radar_size_measure_sigma", ZrSigmas(2))){
		successfulRead = false;
		ZrSigmas(2) = 0;
	}


	Eigen::VectorXd k(nr);
	double posErrorPrDistanceGain = 0;
	if(!nh.getParam("radar_position_error_pr_distance_gain", posErrorPrDistanceGain)){
		successfulRead = false;
	}
	k(0) = posErrorPrDistanceGain;
	k(1) = posErrorPrDistanceGain;


	if(!nh.getParam("radar_size_error_pr_distance_gain", k(2))){
		successfulRead = false;
		k(2) = 0;
	}
	errorPrDistanceGain = k.asDiagonal();


	Eigen::VectorXd biasTimeConstants(nr);
	double posBiasTimeConstant = 1;
	if(!nh.getParam("radar_position_bias_time_constant", posBiasTimeConstant)){
		successfulRead = false;
	}
	biasTimeConstants(0) = posBiasTimeConstant;
	biasTimeConstants(1) = posBiasTimeConstant;

	if(!nh.getParam("radar_size_bias_time_constant", biasTimeConstants(2))){
		successfulRead = false;
		biasTimeConstants(2) = 0;
	}
	Tr = biasTimeConstants.asDiagonal();

	return successfulRead;
}

void detectedObject::set_true_position(gpsPoint truePos){
	X(0) = truePos.longitude;
	X(1) = truePos.latitude;
	lastUpdate = QTime::currentTime();
}

void detectedObject::set_true_COG(double trueCOG){
	X(2) = trueCOG;
	lastUpdate = QTime::currentTime();
}

void detectedObject::set_true_SOG(double trueSOG){
	X(3) = trueSOG;
	lastUpdate = QTime::currentTime();
}

void detectedObject::set_true_cross_section(double trueCS){
	X(4) = trueCS;
	lastUpdate = QTime::currentTime();
}


void detectedObject::set_radar_data(gpsPoint pos, double crossSection){
	set_true_position(pos);
	set_true_cross_section(crossSection);
	lastRADARupdate = QTime::currentTime();
}

void detectedObject::set_AIS_data(double SOG, double COG, double ROT, gpsPoint position){
	Za = Eigen::VectorXd(5);
	Za << SOG, COG, ROT, position.longitude, position.latitude;
	lastAISupdate = QTime::currentTime();
}

void Kalman_project_ahead(	Eigen::VectorXd &X_bar,
							Eigen::VectorXd X_hat, 
							Eigen::VectorXd &f_hat,
							Eigen::MatrixXd &P_bar,
							Eigen::MatrixXd P, 
							Eigen::MatrixXd T_b, 
							double h, 
							Eigen::MatrixXd E, 
							Eigen::MatrixXd Q){
	Eigen::MatrixXd I_12 = Eigen::MatrixXd::Identity(12,12);
	double longDegsPrM = longitude_degs_pr_meter(X_hat(1));
	double latDegsPrM = latitude_degs_pr_meter();

	double u = X_hat(0);
	double psi = deg2rad( X_hat(1) );
	double rot = X_hat(2);
	Eigen::VectorXd b(7);
	b <<  X_hat(5), X_hat(6), X_hat(7), X_hat(8), X_hat(9), X_hat(10), X_hat(11); 
	f_hat << 0, rot, b(6), u*sin(psi)*longDegsPrM, u*cos(psi)*latDegsPrM, T_b*b;
	Eigen::MatrixXd J(12,12);
	J << 	Eigen::MatrixXd::Zero(1,12),
			0, 0, 1, Eigen::MatrixXd::Zero(1,9),
			Eigen::MatrixXd::Zero(1,11), 1, 
			sin(psi)*longDegsPrM, u*cos(psi)*longDegsPrM*M_PI/180, Eigen::MatrixXd::Zero(1,10),
			cos(psi)*latDegsPrM, -u*sin(psi)*latDegsPrM*M_PI/180, Eigen::MatrixXd::Zero(1,10),
			Eigen::MatrixXd::Zero(7,5), T_b;
	Eigen::MatrixXd PHI = I_12 + h*J;

	Eigen::MatrixXd Gamma = h*E;
	X_bar = X_hat + h*f_hat;
	P_bar = PHI*P*PHI.transpose() + Gamma*Q*Gamma;
}

Eigen::VectorXd detectedObject::KalmanFusion(){
	double crossSection = Zr(2);

	Eigen::MatrixXd Ha(5, 12);
	Ha << 	Eigen::MatrixXd::Identity(5, 5), 
			Eigen::MatrixXd::Zero(5, 2), 
			Eigen::MatrixXd::Identity(5,2), 
			Eigen::MatrixXd::Zero(5,3);
	Ha(3,9) = 1;
	Ha(4,10) = 1;
	Eigen::MatrixXd Hr(2,12);
	Hr << Eigen::MatrixXd::Zero(2, 3), Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Zero(2, 5);


	Eigen::VectorXd Tb_trace(7);
	Tb_trace << -0.1, -0.1, -0.05, -0.1, -0.05, -0.05, -0.05;
	Eigen::MatrixXd Tb = Tb_trace.asDiagonal();

	Eigen::VectorXd Zr_sigmas(2);
	Eigen::VectorXd Za_sigmas(5);
	Zr_sigmas << 0.001*longitude_degs_pr_meter(Xa_bar(4)),
				 0.001*latitude_degs_pr_meter(); 
	Eigen::MatrixXd Rr = Zr_sigmas.asDiagonal();
	Za_sigmas << 0.000001, 
				 0.1,  
				 3, 
				 0.001*longitude_degs_pr_meter(Xa_bar(4)), 
				 0.001*latitude_degs_pr_meter(); 
	Eigen::MatrixXd Ra = Za_sigmas.asDiagonal();
	Eigen::VectorXd Q_diag(12);
	Q_diag << 	0.0000001, 
				10, 
				1, 
				1, 
				1, 
				0.5*longitude_degs_pr_meter(Zr(1)), 
				0.5*latitude_degs_pr_meter(), 
				0.5*longitude_degs_pr_meter(Zr(1)), 
				0.2, 
				0.01*longitude_degs_pr_meter(Zr(1)), 
				0.01*latitude_degs_pr_meter(),
				8;
	Eigen::MatrixXd Q = Q_diag.asDiagonal();
	
	Eigen::MatrixXd E(12,12);
	E << 	Eigen::MatrixXd::Zero(5,12),
			Eigen::MatrixXd::Zero(7,5), Eigen::MatrixXd::Identity(7,7);
	Eigen::MatrixXd I_12 = Eigen::MatrixXd::Identity(12,12);

	double longDegsPrM = longitude_degs_pr_meter(Zr(1));
	double latDegsPrM = latitude_degs_pr_meter();
	if( Za != Za_prev && AIS_ON){
		Eigen::MatrixXd K = Pa_bar*Ha.transpose()*(Ha*Pa_bar*Ha.transpose() + Ra ).inverse();
		Xa_hat = Xa_bar + K*(Za - Ha*Xa_bar);
		Pa = (I_12 -K*Ha)*Pa_bar*((I_12 - K*Ha).transpose()) + K*Ra*K.transpose();

		Kalman_project_ahead(Xa_bar, Xa_hat, fa_hat, Pa_bar, Pa, Tb, ha, E, Q);
		Za_prev = Za;

	}
	else if(AIS_ON){
		Xa_hat = Xa_hat + hr*fa_hat;
	}

	Eigen::MatrixXd K = Pr_bar*Hr.transpose()*(Hr*Pr_bar*Hr.transpose() + Rr).inverse();
	Eigen::VectorXd Zr_short(2);
	Zr_short << Zr(0), Zr(1); 
	Xr_hat = Xr_bar + K*(Zr_short - Hr*Xr_bar);
	Pr = (I_12 - K*Hr)*Pr_bar*((I_12 - K*Hr).transpose()) + K*Rr*K.transpose();
	
	Eigen::VectorXd fr_hat(12);
	Kalman_project_ahead(Xr_bar, Xr_hat, fr_hat, Pr_bar, Pr, Tb, hr, E, Q);
	
	Eigen::VectorXd X_hat(nk);
	if(AIS_ON && !radar_ON){
		//qDebug() << "Using AIS only.";
		X_hat = Xa_hat;
		//cout << "Za:\n" << Za << endl;
	}
	else if( radar_ON && !AIS_ON){
		//qDebug() << "Using RADAR only.";
		X_hat = Xr_hat;
	}
	else if(AIS_ON && radar_ON){
		//qDebug() << "Using SENSOR FUSION.";
		Eigen::MatrixXd Pr_w = Pr/hr;
		Eigen::MatrixXd Pa_w = Pa/ha;
		X_hat = (Pr_w.inverse() + Pa_w.inverse()).inverse()*(Pr_w.inverse()*Xr_hat + Pa_w.inverse()*Xa_hat);
		//cout << "Za:\n" << Za << endl;
	}

	//cout << "X_hat:\n" << X_hat << endl;

	double e_longr = (Xr_hat(3) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_latr = (Xr_hat(4) - X(1))/latitude_degs_pr_meter();
	double e_pr = sqrt(pow(e_longr,2) + pow(e_latr,2));
	double e_longa = (Xa_hat(3) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_lata = (Xa_hat(4) - X(1))/latitude_degs_pr_meter();
	double e_pa = sqrt(pow(e_longa,2) + pow(e_lata,2));
	double e_hr = Za(1) - Xr_hat(1);
	double e_ha = Za(1) - Xa_hat(1);
	double e_long = (X_hat(3) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_lat = (X_hat(4) - X(1))/latitude_degs_pr_meter();
	double e_p = sqrt(pow(e_long,2) + pow(e_lat,2));
	double e_h = Za(1) - X_hat(1);
	
	/*
	cout << "Radar  position error: " << e_pr << endl;
	cout << "Radar  heading  error: " << e_hr << endl;
	cout << "AIS    position error: " << e_pa << endl;
	cout << "AIS    heading  error: " << e_ha << endl;
	cout << "Fusion position error: " << e_p << endl;
	cout << "Fusion heading  error: " << e_h << endl;
	cout << "Za:\n" << Za << endl;
	cout << "X_hat:\n" << X_hat << endl;
	*/

	cout << "Fusion position error: " << e_p << endl;

	Eigen::VectorXd X_hat_short = Eigen::VectorXd::Zero(n);
	double positionErrorTreshold = 50;
	if (abs(e_p) > positionErrorTreshold )
	{
		if (AIS_ON){
			X_hat_short << Za(3), Za(4), Za(1), Za(0), crossSection;
		}
		else if(radar_ON){
			X_hat_short << Zr(0), Zr(1), 0, 0, crossSection;
		}
	}
	else{
		X_hat_short << X_hat(3), X_hat(4), X_hat(1), X_hat(0), crossSection;
	}
	return X_hat_short;
}
















