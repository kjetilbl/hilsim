
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

	// ros::AsyncSpinner spinner(1);
	// spinner.start();
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
	/*
	qDebug() << "\n---------------------Publish USV AIS message---------------------";
	USVnavData.print_data();
	qDebug() << "-----------------------------------------------------------------\n";
	*/
}


void sensorSim::publish_detected_targets()
{
	std::lock_guard<std::mutex> lock(m);
	qDebug() << "publish_detected_targets start";

	vector<string> outdatedObstacles;

	// Publish obstacle position info and find outdated obstacles
	for( auto & obst : unidentifiedObjects )
	{
		string obstID = obst.first;
		if( obst.second.msecs_since_last_update() > 3000)
		{
			outdatedObstacles.push_back(obstID);
		}
		else
		{
			qDebug() << 1;
			double distanceToObject = distance_m(USVnavData.get_position(), obst.second.get_true_position());
			qDebug() << 2;
			obst.second.generate_radar_measurement(distanceToObject);
			qDebug() << 3;
			simulator_messages::detectedTarget dt = obst.second.makeDTmsg();
			qDebug() << 4;

			detectedTargetPub.publish(dt);
			//ros::spinOnce();
			qDebug() << 5;
			/*
			QTime now = QTime::currentTime();
			qDebug() 	<< "Publishing detected target" << obst.second.get_target_number() << "at time" 
				<< now.hour() << ":" << now.minute() << ":" << now.second() << ":" << now.msec();
			*/
		}
	}
	// Delete old obstacles
	for ( auto const& ID : outdatedObstacles ) 
	{
    	unidentifiedObjects.erase(ID);
	}
	qDebug() << "publish_detected_targets end";
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

	qDebug() << "obstacle_update_parser start";
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
		unidentifiedObjects[objectID].set_true_position(obstPos);
		// unidentifiedObjects[objectID].set_true_COG(COG);
		unidentifiedObjects[objectID].set_true_cross_section(crossSection);
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
	qDebug() << "obstacle_update_parser end";
	
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
	double crossSection = pow(150,2);
	map<string, detectedObject>::iterator it = unidentifiedObjects.find(objectID);
	if( it != unidentifiedObjects.end() ) // Object already detected
	{
		crossSection = unidentifiedObjects[objectID].get_true_CS();
		unidentifiedObjects[objectID].AIS_ON = true;
		unidentifiedObjects[objectID].set_AIS_data(SOG, COG, position);

		if( is_within_visibility(position, crossSection, objectID) ){
			unidentifiedObjects[objectID].radar_ais_fusion = true;
		}
		else{
			unidentifiedObjects[objectID].radar_ais_fusion = false;
		}
		/*unidentifiedObjects[objectID].set_true_SOG(SOG);
		if( !is_within_visibility(position, crossSection, objectID) ){
			unidentifiedObjects[objectID].set_true_position(position);
			unidentifiedObjects[objectID].set_true_COG(COG);
		}
		else{
			unidentifiedObjects[objectID].noiseEnabled = false;
			// Dont update nav data, trust radar and lidar.
		}*/
	}else
	{
		unidentifiedObjects[objectID] = detectedObject(*nh, objectDescriptor, position, COG, SOG, crossSection);
		unidentifiedObjects[objectID].noiseEnabled = false;
	}
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
	int n = 5;
	X = Eigen::VectorXd(n);
	Xm = Eigen::VectorXd(n);
	b = Eigen::VectorXd::Zero(n);
	Td = Eigen::MatrixXd::Zero(n, n); // discretize at each time step (varying sample time)

	Za = Eigen::VectorXd(4);
	Za << trueSOG, trueCOG, truePosition.longitude, truePosition.latitude;
	Za_prev = Eigen::VectorXd::Zero(4);
	Xr_bar = Eigen::VectorXd(10);
	Xr_bar << Za, Eigen::VectorXd::Zero(6);
	Xr_hat = Xr_bar;
	Xa_bar = Xr_bar;
	Xa_hat = Xr_bar;
	cout << "First Xa_hat:\n" << Xa_hat << endl;
	Pa_bar = 10*Eigen::MatrixXd::Identity(10,10);
	Pr_bar = 100*Eigen::MatrixXd::Identity(10,10);
	Pr_bar(0,0) = 1000000;
	Pr_bar(1,1) = 1000000;
	double u = Xa_hat(0);
	double psi = deg2rad( Xa_hat(1) );
	fa_hat = Eigen::VectorXd(10);
	fa_hat << 0, 0, u*sin(psi)*longitude_degs_pr_meter(truePosition.latitude), u*cos(psi)*latitude_degs_pr_meter(), 0, 0, 0, 0, 0, 0;

	
	if(!read_sensor_config(nh, truePosition)){
		qDebug() << "detectedObject read sensor config failed.";
		exit(-1);
	}
	update_true_states(truePosition, trueCOG, trueSOG, trueCrossSection);
}

Eigen::VectorXd detectedObject::generate_radar_measurement( double distanceFromUSV_m ){
	if (AIS_ON && radar_ais_fusion == false){
		double cs = get_estimated_CS();
		Xm << Za(2), Za(3), Za(0), Za(1), cs; 
		return Xm;
	}

	int n = Xm.rows();
	static std::default_random_engine randomGenerator;
	static std::normal_distribution<double> gaussianWhiteNoise(0,1);

	if (firstTimeParamEstimate)
	{
		for (int i = 0; i < n; i++)
		{
			b(i) = gaussianWhiteNoise(randomGenerator)*biasSigmas(i)*25;
		}
		firstTimeParamEstimate = false;
		lastEstimateTime = QTime::currentTime();
	}
	
	QTime now = QTime::currentTime();
	double dt = (double)(lastEstimateTime.msecsTo( now ))/1000;

	Eigen::VectorXd e(n);
	Eigen::VectorXd w(n);
	Eigen::VectorXd v(n);
	for (int i = 0; i < n; i++)
	{
		Td(i,i) = exp(-1/T(i,i)*dt);
		w(i) = gaussianWhiteNoise(randomGenerator)*biasSigmas(i);
		v(i) = gaussianWhiteNoise(randomGenerator)*measureSigmas(i);
	}

	// Make artificial deviations
	b = Td*b + w;
	e = b + v;
	// e = errorPrDistanceGain * distanceFromUSV_m * e;
	e(3) = max(-X(3), (double)e(3));
	e(4) = max(-X(4) + 1, (double)e(4));

	// Update artificial measurements:
	Xm = X + e;

	// Keep estimated COG in valid range:
	while (Xm(2) < 0)
	{
		Xm(2) += 360.0;
	}
	while (Xm(2) > 360.0)
	{
		Xm(2) -= 360.0;
	}

	lastEstimateTime = now;

	if(radar_ais_fusion == true){
		Eigen::VectorXd Zr(2);
		Zr << Xm(0), Xm(1);
		Eigen::VectorXd X_hat(4);
		X_hat = KalmanFusion(Zr, 0.1, 2);
		double cs = Xm(4);
		Xm << X_hat(2), X_hat(3), X_hat(1), X_hat(0), cs;
	}

	return Xm;
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
	int n = 5;
	biasSigmas = Eigen::VectorXd(n);
	double positionBiasSigma = 0;
	if(!nh.getParam("DTM_position_bias_sigma", positionBiasSigma)){
		successfulRead = false;
	}
	biasSigmas(0) = positionBiasSigma*longitude_degs_pr_meter(truePosition.latitude);
	biasSigmas(1) = positionBiasSigma*latitude_degs_pr_meter();

	if(!nh.getParam("DTM_COG_bias_sigma", biasSigmas(2))){
		successfulRead = false;
		biasSigmas(2) = 0;
	}

	if(!nh.getParam("DTM_SOG_bias_sigma", biasSigmas(3))){
		successfulRead = false;
		biasSigmas(3) = 0;
	}

	if(!nh.getParam("DTM_size_bias_sigma", biasSigmas(4))){
		successfulRead = false;
		biasSigmas(4) = 0;
	}

	measureSigmas = Eigen::VectorXd(n);
	double positionMeasureSigma = 0;
	if(!nh.getParam("DTM_position_measure_sigma", positionMeasureSigma)){
		successfulRead = false;
	}
	measureSigmas(0) = positionMeasureSigma*longitude_degs_pr_meter(truePosition.latitude);
	measureSigmas(1) = positionMeasureSigma*latitude_degs_pr_meter();

	if(!nh.getParam("DTM_COG_measure_sigma", measureSigmas(2))){
		successfulRead = false;
		measureSigmas(2) = 0;
	}

	if(!nh.getParam("DTM_SOG_measure_sigma", measureSigmas(3))){
		successfulRead = false;
		measureSigmas(3) = 0;
	}

	if(!nh.getParam("DTM_size_measure_sigma", measureSigmas(4))){
		successfulRead = false;
		measureSigmas(4) = 0;
	}


	Eigen::VectorXd k(n);
	double posErrorPrDistanceGain = 0;
	if(!nh.getParam("DTM_position_error_pr_distance_gain", posErrorPrDistanceGain)){
		successfulRead = false;
	}
	k(0) = posErrorPrDistanceGain;
	k(1) = posErrorPrDistanceGain;

	if(!nh.getParam("DTM_COG_error_pr_distance_gain", k(2))){
		successfulRead = false;
		k(2) = 0;
	}

	if(!nh.getParam("DTM_SOG_error_pr_distance_gain", k(3))){
		successfulRead = false;
		k(3) = 0;
	}

	if(!nh.getParam("DTM_size_error_pr_distance_gain", k(4))){
		successfulRead = false;
		k(4) = 0;
	}
	errorPrDistanceGain = k.asDiagonal();


	Eigen::VectorXd biasTimeConstants(n);
	double posBiasTimeConstant = 1;
	if(!nh.getParam("DTM_position_bias_time_constant", posBiasTimeConstant)){
		successfulRead = false;
	}
	biasTimeConstants(0) = posBiasTimeConstant;
	biasTimeConstants(1) = posBiasTimeConstant;

	if(!nh.getParam("DTM_COG_bias_time_constant", biasTimeConstants(2))){
		successfulRead = false;
		biasTimeConstants(2) = 0;
	}

	if(!nh.getParam("DTM_SOG_bias_time_constant", biasTimeConstants(3))){
		successfulRead = false;
		biasTimeConstants(3) = 0;
	}

	if(!nh.getParam("DTM_size_bias_time_constant", biasTimeConstants(4))){
		successfulRead = false;
		biasTimeConstants(4) = 0;
	}
	T = biasTimeConstants.asDiagonal();

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

void detectedObject::set_AIS_data(double SOG, double COG, gpsPoint position){
	Za = Eigen::VectorXd(4);
	Za << SOG, COG, position.longitude, position.latitude;
}

Eigen::VectorXd detectedObject::KalmanFusion(Eigen::VectorXd Zr, double hr, double ha){
	if(Zr.rows() != 2){
		cout << "Invalid Zr:\n" << Zr << endl;
		exit(-1);
	}

	Eigen::MatrixXd Ha(4, 10);
	Ha << Eigen::MatrixXd::Identity(4, 4), Eigen::MatrixXd::Zero(4, 2), Eigen::MatrixXd::Identity(4,4);
	Eigen::MatrixXd Hr(2,10);
	Hr << Eigen::MatrixXd::Zero(2, 2), Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Zero(2, 4);


	Eigen::VectorXd Tb_trace(6);
	Tb_trace << -0.1, -0.1, -0.05, -0.1, -0.05, -0.05;
	Eigen::MatrixXd Tb = Tb_trace.asDiagonal();

	Eigen::VectorXd Zr_sigmas(2);
	Eigen::VectorXd Za_sigmas(4);
	Zr_sigmas << 0.01*longitude_degs_pr_meter(Xa_bar(3)),
				 0.01*latitude_degs_pr_meter(); 
	Eigen::MatrixXd Rr = Zr_sigmas.asDiagonal();
	Za_sigmas << 0.000001, 
				 0.000001, 
				 0.000001*longitude_degs_pr_meter(Xa_bar(3)), 
				 0.000001*latitude_degs_pr_meter(); 
	Eigen::MatrixXd Ra = Za_sigmas.asDiagonal();
	Eigen::VectorXd Q_diag(10);
	Q_diag << 	0.0000001, 
				1, 
				0.0000001, 
				0.0000001, 
				0.5*longitude_degs_pr_meter(Zr(1)), 
				0.5*latitude_degs_pr_meter(), 
				0.5*longitude_degs_pr_meter(Zr(1)), 
				0.2, 
				0.01*longitude_degs_pr_meter(Zr(1)), 
				0.01*latitude_degs_pr_meter();
	Eigen::MatrixXd Q = Q_diag.asDiagonal();
	
	Eigen::MatrixXd E(10,10);
	E << 	Eigen::MatrixXd::Zero(4,10),
			Eigen::MatrixXd::Zero(6,4), Eigen::MatrixXd::Identity(6,6);
	Eigen::MatrixXd I_10 = Eigen::MatrixXd::Identity(10,10);

	double longDegsPrM = longitude_degs_pr_meter(Zr(1));
	double latDegsPrM = latitude_degs_pr_meter();
	if( Za != Za_prev){
		Eigen::MatrixXd K = Pa_bar*Ha.transpose()*(Ha*Pa_bar*Ha.transpose() + Ra ).inverse();
		Xa_hat = Xa_bar + K*(Za - Ha*Xa_bar);
		Pa = (I_10 -K*Ha)*Pa_bar*((I_10 - K*Ha).transpose()) + K*Ra*K.transpose();

		// Jacobian:
		double u = Xa_hat(0);
		double psi = deg2rad( Xa_hat(1) );
		Eigen::VectorXd b(6);
		b <<  Xa_hat(4), Xa_hat(5), Xa_hat(6), Xa_hat(7), Xa_hat(8), Xa_hat(9); 
		fa_hat << 0, 0, u*sin(psi)*longDegsPrM, u*cos(psi)*latDegsPrM, Tb*b;
		Eigen::MatrixXd Ja(10,10);
		Ja << 	Eigen::MatrixXd::Zero(2,10),
				sin(psi)*longDegsPrM, u*cos(psi)*longDegsPrM*M_PI/180, Eigen::MatrixXd::Zero(1,8),
				cos(psi)*latDegsPrM, -u*sin(psi)*latDegsPrM*M_PI/180, Eigen::MatrixXd::Zero(1,8),
				Eigen::MatrixXd::Zero(6,4), Tb;

		Eigen::MatrixXd PHIa = I_10 + ha*Ja;

		// Project ahead:
		Eigen::MatrixXd Gamma = ha*E;
		Xa_bar = Xa_hat + ha*fa_hat;
		Pa_bar = PHIa*Pa*PHIa.transpose() + Gamma*Q*Gamma;
		Za_prev = Za;

	}
	else{
		Xa_hat = Xa_hat + hr*fa_hat;
	}

	Eigen::MatrixXd K = Pr_bar*Hr.transpose()*(Hr*Pr_bar*Hr.transpose() + Rr).inverse();
	Xr_hat = Xr_bar + K*(Zr - Hr*Xr_bar);
	Pr = (I_10 - K*Hr)*Pr_bar*((I_10 - K*Hr).transpose()) + K*Rr*K.transpose();
	


	// Jacobian:
	double u = Xr_hat(0);
	double psi = deg2rad( Xr_hat(1) );
	Eigen::VectorXd b(6);
	b <<  Xr_hat(4), Xr_hat(5), Xr_hat(6), Xr_hat(7), Xr_hat(8), Xr_hat(9);
	Eigen::VectorXd fr_hat(10);
	fr_hat << 0, 0, u*sin(psi)*longDegsPrM, u*cos(psi)*latDegsPrM, Tb*b;
	Eigen::MatrixXd Jr(10,10);
	Jr << 	Eigen::MatrixXd::Zero(2,10),
			sin(psi)*longDegsPrM, u*cos(psi)*longDegsPrM*M_PI/180, Eigen::MatrixXd::Zero(1,8),
			cos(psi)*latDegsPrM, -u*sin(psi)*latDegsPrM*M_PI/180, Eigen::MatrixXd::Zero(1,8),
			Eigen::MatrixXd::Zero(6,4), Tb;
	Eigen::MatrixXd PHIr = I_10 + hr*Jr;

	// Project ahead:
	Eigen::MatrixXd Gamma = hr*E;
	Xr_bar = Xr_hat + hr*fr_hat;
	Pr_bar = PHIr*Pr*PHIr.transpose() + Gamma*Q*Gamma;

	Eigen::MatrixXd Pr_w = Pr/hr;
	Eigen::MatrixXd Pa_w = Pa/ha;

	Eigen::VectorXd X_hat = (Pr_w.inverse() + Pa_w.inverse()).inverse()*(Pr_w.inverse()*Xr_hat + Pa_w.inverse()*Xa_hat);

	double e_longr = (Xr_hat(2) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_latr = (Xr_hat(3) - X(1))/latitude_degs_pr_meter();
	double e_pr = sqrt(pow(e_longr,2) + pow(e_latr,2));
	double e_longa = (Xa_hat(2) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_lata = (Xa_hat(3) - X(1))/latitude_degs_pr_meter();
	double e_pa = sqrt(pow(e_longa,2) + pow(e_lata,2));
	double e_hr = Za(1) - Xr_hat(1);
	double e_ha = Za(1) - Xa_hat(1);
	double e_long = (X_hat(2) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_lat = (X_hat(3) - X(1))/latitude_degs_pr_meter();
	double e_p = sqrt(pow(e_long,2) + pow(e_lat,2));
	double e_h = Za(1) - X_hat(1);
	cout << "Radar  position error: " << e_pr << endl;
	cout << "Radar  heading  error: " << e_hr << endl;
	cout << "AIS    position error: " << e_pa << endl;
	cout << "AIS    heading  error: " << e_ha << endl;
	cout << "Fusion position error: " << e_p << endl;
	cout << "Fusion heading  error: " << e_h << endl;
	cout << "Za:\n" << Za << endl;
	cout << "X_hat:\n" << X_hat << endl;
	cout << "Pr_w:\n" << (Pr_w.inverse() + Pa_w.inverse()).inverse()*Pr_w.inverse() << endl;

	return X_hat;
}
















