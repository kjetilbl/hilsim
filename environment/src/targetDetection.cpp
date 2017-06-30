
#include "targetDetection.h"
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
//----------------------------------------targetDetectionModule----------------------------------------
//----------------------------------------------------------------------------------------


targetDetectionModule::targetDetectionModule(ros::NodeHandle *n, QThread *parent) : QThread(parent)
{
	nh = n;
	if( !read_sensor_config() ){
		qDebug() << "targetDetectionModule read sensor config failed.";
		exit(0);
	}
}

targetDetectionModule::~targetDetectionModule()
{
	delete DTtimer;
}

void targetDetectionModule::run()
{
	USVnavData = navData(316001245, 123.8777500, 49.2002817, 19.6, 235.0);
	ros::Subscriber USVupdateSub = nh->subscribe("sensors/gps", 1000, &targetDetectionModule::USV_gps_parser, this);
	ros::Subscriber obstUpdateSub = nh->subscribe("/simObject/position", 1000, &targetDetectionModule::obstacle_update_parser, this);
	ros::Subscriber AISsub = nh->subscribe("sensors/ais", 1000, &targetDetectionModule::AIS_parser, this);
	detectedTargetPub = nh->advertise<simulator_messages::detectedTarget>("sensors/target_detection", 1000);

	DTtimer = new QTimer();
	QObject::connect( DTtimer, SIGNAL(timeout()), this, SLOT(publish_detected_targets()) );
	DTtimer->start(100);

	QThread::exec();
}

bool targetDetectionModule::read_sensor_config(){
	bool successfulRead = true;
	if(!nh->getParam("radar_range", radarRange)){
		radarRange = 200;
		successfulRead = false;
	}
	if(!nh->getParam("lidar_range", lidarRange)){
		radarRange = 100;
		successfulRead = false;
	}
	return successfulRead;
}

void targetDetectionModule::publish_detected_targets()
{
	std::lock_guard<std::mutex> lock(m);
	vector<string> inactiveTargets;

	// Publish obstacle position info and find outdated obstacles
	for( auto & target : detectedTargets )
	{
		string targetID = target.first;
		if( !target.second.is_active() )
		{
			inactiveTargets.push_back(targetID);
		}
		else
		{
			double distanceToObject = distance_m(USVnavData.get_position(), target.second.get_true_position());
			target.second.estimate_states(distanceToObject);
			simulator_messages::detectedTarget dt = target.second.make_DT_msg();

			detectedTargetPub.publish(dt);
		}
	}
	// Delete old obstacles
	for ( auto const& ID : inactiveTargets ) 
	{
    	detectedTargets.erase(ID);
	}
}


void targetDetectionModule::USV_gps_parser(const simulator_messages::Gps::ConstPtr& USVgpsMsg)
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

void targetDetectionModule::obstacle_update_parser(const simulator_messages::obstacleUpdate::ConstPtr& obstUpdateMsg)
{
	std::lock_guard<std::mutex> lock(m);

	string objectDescriptor = obstUpdateMsg->objectDescriptor;
	gpsPoint obstPos(obstUpdateMsg->longitude, obstUpdateMsg->latitude);
	double radius = obstUpdateMsg->radius;
	double COG = obstUpdateMsg->heading;
	string objectID = obstUpdateMsg->objectID;

	if( !is_within_range(obstPos, radarRange) && !is_within_range(obstPos, lidarRange) ){
		return;
	}else if ( !is_visible(obstPos, objectID) )
	{
		return;
	}

	map<string, detectedObject>::iterator it = detectedTargets.find(objectID);
	if( it != detectedTargets.end() ) // Object already detected
	{
		detectedTargets[objectID].set_true_position(obstPos);
		detectedTargets[objectID].set_true_radius(radius);
	}else
	{
		if (objectDescriptor == "fixed_obstacle")
		{
			detectedTargets[objectID] = detectedObject(*nh, objectDescriptor, obstPos, COG, 0, radius);
		}
		else if (objectDescriptor == "ship")
		{
			detectedTargets[objectID] = detectedObject(*nh, "vessel", obstPos, COG, 5, radius);
		}
	}
	if (is_within_range(obstPos, radarRange))
	{
		detectedTargets[objectID].set_radar_active(true);
	}
	if (is_within_range(obstPos, lidarRange))
	{
		detectedTargets[objectID].set_lidar_active(true);
	}
}

void targetDetectionModule::AIS_parser(const simulator_messages::AIS::ConstPtr& AISmsg)
{
	std::lock_guard<std::mutex> lock(m);

	string objectDescriptor = "vessel";
	string objectID = "AIS_user_" + to_string(AISmsg->MMSI);
	gpsPoint position(AISmsg->longitude, AISmsg->latitude);

	double COG = AISmsg->COG;
	double SOG = AISmsg->SOG/1.94384449; // [m/s]
	double ROT = AISmsg->ROT*60; // [deg/sec]
	double radius = 150;
	map<string, detectedObject>::iterator it = detectedTargets.find(objectID);
	if( it == detectedTargets.end() )
	{
		detectedTargets[objectID] = detectedObject(*nh, objectDescriptor, position, COG, SOG, radius);
	}
	detectedTargets[objectID].set_AIS_data(SOG, COG, ROT, position);
}

bool targetDetectionModule::is_within_range(gpsPoint pos, double range){
	gpsPointStamped usvPos = USVnavData.get_position();
	double distToNewPos = distance_m(usvPos, pos);
	if (distToNewPos > range)
	{
		return false;
	}
	return true;
}

bool targetDetectionModule::is_visible(gpsPoint newPos, string objectID = "unknown"){
	gpsPointStamped usvPos = USVnavData.get_position();
	double distToNewPos = distance_m(usvPos, newPos);

	// Check if behind other objects
	for( auto & mapPair : detectedTargets ){
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
	return true;
}

detectedObject::detectedObject(){}

detectedObject::detectedObject(	ros::NodeHandle nh,
								string objectDescriptor,
								gpsPoint truePosition, 
								double trueCOG, 
								double trueSOG, 
								double trueRadius)
{
	descriptor = objectDescriptor;
	targetNumber = targetIterator++;
	X = Eigen::VectorXd(n);
	Xe = Eigen::VectorXd(n);
	br = Eigen::VectorXd::Zero(nr);
	bl = Eigen::VectorXd::Zero(nr);
	Za = Eigen::VectorXd(na);
	Za << truePosition.longitude, truePosition.latitude, trueCOG, trueSOG, 0;
	Za_prev = Eigen::VectorXd::Zero(na);
	Xr_bar = Eigen::VectorXd(nk);
	Xr_bar << Za, trueRadius, Eigen::VectorXd::Zero(nk - na - 1);
	Xr_hat = Xr_bar;
	Xl_bar = Xr_bar;
	Xl_hat = Xr_bar;
	Xa_bar = Xr_bar;
	Xa_hat = Xr_bar;

	Ha = Eigen::MatrixXd(5, 19);
	Ha << 	Eigen::MatrixXd::Identity(5, 5), 
			Eigen::MatrixXd::Zero(5, 3), 
			Eigen::MatrixXd::Identity(5,5), 
			Eigen::MatrixXd::Zero(5,6);
	Ha(4,12) = 0;
	Hr = Eigen::MatrixXd(3,19);
	Hr << Eigen::MatrixXd::Identity(3, 2), Eigen::MatrixXd::Zero(3, 11), Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Zero(3, 3);
	Hr(2,5) = 1;
	Hl = Hr;
	
	Pa_bar = 10*Eigen::MatrixXd::Identity(nk,nk);
	Pa_bar(0,0) = 1*longitude_degs_pr_meter(truePosition.latitude);
	Pa_bar(1,1) = 1*latitude_degs_pr_meter();
	Pa_bar(2,2) = 1; // Heading
	Pa_bar(3,3) = 1;  // Speed
	Pa_bar(4,4) = 5;  // ROT
	Pa_bar(5,5) = 10; // CS
	Pa = Pa_bar;

	Pr_bar = 10*Eigen::MatrixXd::Identity(nk,nk);
	Pr_bar(0,0) = 100*longitude_degs_pr_meter(truePosition.latitude);
	Pr_bar(1,1) = 100*latitude_degs_pr_meter();
	Pr_bar(2,2) = 100; 		// Heading
	Pr_bar(3,3) = 1000000;  // Speed
	Pr_bar(4,4) = 50;  		// ROT
	Pr_bar(5,5) = 1000; 	// CS
	Pr = Pr_bar;

	Pl_bar = 10*Eigen::MatrixXd::Identity(nk,nk);
	Pl_bar(0,0) = 100*longitude_degs_pr_meter(truePosition.latitude);
	Pl_bar(1,1) = 100*latitude_degs_pr_meter();
	Pl_bar(2,2) = 100; 		// Heading
	Pl_bar(3,3) = 1000000;  // Speed
	Pl_bar(4,4) = 50;  		// ROT
	Pl_bar(5,5) = 10; 		// CS
	Pl = Pl_bar;

	Eigen::VectorXd Q_diag(nk);
	Q_diag << 	0.005,
				0.005,
				0.0001,
				0.0001,
				0.0001,
				0.0001,
				10000000000, 
				0.1, 
				0.5*longitude_degs_pr_meter(truePosition.latitude), 
				0.5*latitude_degs_pr_meter(), 
				0.5,
				0.5,
				0.5, 
				1,
				1,
				1,
				1,
				1,
				1;
	Q = Q_diag.asDiagonal();

	E = Eigen::MatrixXd(19,19);
	E << 	Eigen::MatrixXd::Zero(6,19),
			Eigen::MatrixXd::Zero(13,6), Eigen::MatrixXd::Identity(13,13);
	
	double u = Xa_hat(3);
	double psi = deg2rad( Xa_hat(2) );
	double rot = deg2rad( Xa_hat(4) );
	fa_hat = Eigen::VectorXd(nk);
	fa_hat << u*sin(psi)*longitude_degs_pr_meter(truePosition.latitude), u*cos(psi)*latitude_degs_pr_meter(), rot, 0, 0, 0, Eigen::VectorXd::Zero(nk-6);
	if(!read_sensor_config(nh, truePosition)){
		qDebug() << "detectedObject read sensor config failed.";
		exit(-1);
	}

	update_true_states(truePosition, trueCOG, trueSOG, trueRadius);
	lastAISupdate = QTime::currentTime();
	lastRadarUpdate = QTime::currentTime();
	lastLidarUpdate = QTime::currentTime();
	lastRadarEstimateTime = QTime::currentTime();
	lastlidarEstimateTime = QTime::currentTime();
}

Eigen::VectorXd detectedObject::estimate_states( double distanceFromUSV_m ){
	QTime now = QTime::currentTime();
	if (lastAISupdate.msecsTo(now) > 5000)
	{
		AIS_ON = false;
	}
	if (lastRadarUpdate.msecsTo(now) > 2000)
	{
		radar_ON = false;
	}
	if (lastLidarUpdate.msecsTo(now) > 2000)
	{
		lidar_ON = false;
	}
	if( radar_ON ){
		Zr = generate_radar_measurement(distanceFromUSV_m);
	}
	if( lidar_ON ){
		Zl = generate_lidar_measurement(distanceFromUSV_m);
	}
	Xe = KalmanFusion();
	return Xe;
}

Eigen::VectorXd detectedObject::generate_radar_measurement( double distanceFromUSV_m ){
	static std::default_random_engine randomGenerator;
	static std::normal_distribution<double> gaussianWhiteNoise(0,1);
	
	QTime now = QTime::currentTime();
	double dt = (double)(lastRadarEstimateTime.msecsTo( now ))/1000;

	Eigen::VectorXd e(nr);
	Eigen::VectorXd w(nr);
	Eigen::VectorXd v(nr);
	Eigen::MatrixXd Tdr = Eigen::MatrixXd::Zero(nr, nr);
	for (int i = 0; i < nr; i++)
	{
		Tdr(i,i) = exp(Tb(i+7, i+7)*dt);
		w(i) = gaussianWhiteNoise(randomGenerator)*brSigmas(i);
		v(i) = gaussianWhiteNoise(randomGenerator)*Ra(i,i);
	}
	// Make artificial deviations
	br = Tdr*br + w;
	e = br + v;
	e = errorPrDistanceGain * distanceFromUSV_m * e;
	e(2) = max(-X(4) + 2, (double)e(2));


	// Make artificial measurements:
	Eigen::VectorXd Z(nr);
	Eigen::VectorXd trueStates(nr);
	trueStates << X(0), X(1), X(4);
	Z = trueStates + e;
	lastRadarEstimateTime = now;

	return Z;
}

Eigen::VectorXd detectedObject::generate_lidar_measurement( double distanceFromUSV_m ){
	static std::default_random_engine randomGenerator;
	static std::normal_distribution<double> gaussianWhiteNoise(0,1);
	
	QTime now = QTime::currentTime();
	double dt = (double)(lastlidarEstimateTime.msecsTo( now ))/1000;

	Eigen::VectorXd e(nr);
	Eigen::VectorXd w(nr);
	Eigen::VectorXd v(nr);
	Eigen::MatrixXd Tdl = Eigen::MatrixXd::Zero(nr, nr);
	for (int i = 0; i < nr; i++)
	{
		Tdl(i,i) = exp(Tb(i+10, i+10)*dt);
		w(i) = gaussianWhiteNoise(randomGenerator)*blSigmas(i);
		v(i) = gaussianWhiteNoise(randomGenerator)*Rl(i,i);
	}
	// Make artificial deviations
	bl = Tdl*bl + w;
	e = bl + v;
	e = errorPrDistanceGain * distanceFromUSV_m * e;
	e(2) = max( -X(4)+2, (double)e(2));


	// Make artificial measurements:
	Eigen::VectorXd Z(nr);
	Eigen::VectorXd trueStates(nr);
	trueStates << X(0), X(1), X(4);
	Z = trueStates + e;

	lastlidarEstimateTime = now;

	return Z;
}

void detectedObject::update_true_states(gpsPoint pos, double COG, double SOG, double radius){
	set_true_position(pos);
	set_true_COG(COG);
	set_true_SOG(SOG);
	set_true_radius(radius);
}

simulator_messages::detectedTarget detectedObject::make_DT_msg(){
	simulator_messages::detectedTarget dt;
	dt.targetID = get_target_number();
	dt.objectDescriptor = descriptor;
	dt.longitude = get_estimated_position().longitude;
	dt.latitude = get_estimated_position().latitude;
	dt.COG = round( get_estimated_COG() );
	dt.SOG = round( get_estimated_SOG() * 1.94384449 ); // knots
	dt.radius = get_estimated_radius()*2;
	return dt;
}

bool detectedObject::read_sensor_config(ros::NodeHandle nh, gpsPoint truePosition){
	bool successfulRead = true;

	blSigmas = Eigen::VectorXd(nr);
	double positionBiasSigma = 1;
	if(!nh.getParam("lidar_position_bias_sigma", positionBiasSigma)){
		successfulRead = false;
	}
	blSigmas(0) = positionBiasSigma*longitude_degs_pr_meter(truePosition.latitude);
	blSigmas(1) = positionBiasSigma*latitude_degs_pr_meter();


	if(!nh.getParam("lidar_size_bias_sigma", blSigmas(2))){
		successfulRead = false;
		blSigmas(2) = 0;
	}

	Eigen::VectorXd ZlSigmas = Eigen::VectorXd(nr);
	double positionMeasureSigma = 0;
	if(!nh.getParam("lidar_position_measure_sigma", positionMeasureSigma)){
		successfulRead = false;
	}
	ZlSigmas(0) = positionMeasureSigma*longitude_degs_pr_meter(truePosition.latitude);
	ZlSigmas(1) = positionMeasureSigma*latitude_degs_pr_meter();

	if(!nh.getParam("lidar_size_measure_sigma", ZlSigmas(2))){
		successfulRead = false;
		ZlSigmas(2) = 0;
	}

	Rl = ZlSigmas.asDiagonal();

	brSigmas = Eigen::VectorXd(nr);
	positionBiasSigma = 1;
	if(!nh.getParam("radar_position_bias_sigma", positionBiasSigma)){
		successfulRead = false;
	}
	brSigmas(0) = positionBiasSigma*longitude_degs_pr_meter(truePosition.latitude);
	brSigmas(1) = positionBiasSigma*latitude_degs_pr_meter();


	if(!nh.getParam("radar_size_bias_sigma", brSigmas(2))){
		successfulRead = false;
		brSigmas(2) = 0;
	}

	Eigen::VectorXd ZrSigmas = Eigen::VectorXd(nr);
	positionMeasureSigma = 1;
	if(!nh.getParam("radar_position_measure_sigma", positionMeasureSigma)){
		successfulRead = false;
	}
	ZrSigmas(0) = positionMeasureSigma*longitude_degs_pr_meter(truePosition.latitude);
	ZrSigmas(1) = positionMeasureSigma*latitude_degs_pr_meter();

	if(!nh.getParam("radar_size_measure_sigma", ZrSigmas(2))){
		successfulRead = false;
		ZrSigmas(2) = 0;
	}

	Rr = ZrSigmas.asDiagonal();

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

	// AIS params:
	Eigen::VectorXd ZaSigmas(n);
	positionMeasureSigma = 0.01;
	if(!nh.getParam("AIS_position_measure_sigma", positionMeasureSigma)){
		successfulRead = false;
	}
	ZaSigmas(0) = positionMeasureSigma*longitude_degs_pr_meter(truePosition.latitude);
	ZaSigmas(1) = positionMeasureSigma*latitude_degs_pr_meter();

	if(!nh.getParam("AIS_COG_measure_sigma", ZaSigmas(2))){
		successfulRead = false;
		ZaSigmas(2) = 0;
	}

	if(!nh.getParam("AIS_SOG_measure_sigma", ZaSigmas(3))){
		successfulRead = false;
		ZaSigmas(3) = 0;
	}

	if(!nh.getParam("AIS_ROT_measure_sigma", ZaSigmas(4))){
		successfulRead = false;
		ZaSigmas(4) = 0;
	}
	Ra = ZaSigmas.asDiagonal();


	// Bias time constants:
	Eigen::VectorXd biasTimeConstants(nb);
	double posBiasTimeConstant = 1;
	biasTimeConstants(0) = 10;
	biasTimeConstants(1) = 100;
	if(!nh.getParam("AIS_position_bias_time_constant", posBiasTimeConstant)){
		successfulRead = false;
	}
	biasTimeConstants(2) = posBiasTimeConstant;
	biasTimeConstants(3) = posBiasTimeConstant;

	if(!nh.getParam("AIS_COG_bias_time_constant", biasTimeConstants(4))){
		successfulRead = false;
		biasTimeConstants(4) = 1;
	}

	if(!nh.getParam("AIS_SOG_bias_time_constant", biasTimeConstants(5))){
		successfulRead = false;
		biasTimeConstants(5) = 1;
	}

	if(!nh.getParam("AIS_ROT_bias_time_constant", biasTimeConstants(6))){
		successfulRead = false;
		biasTimeConstants(6) = 1;
	}

	if(!nh.getParam("radar_position_bias_time_constant", biasTimeConstants(7))){
		successfulRead = false;
		biasTimeConstants(7) = 1;
	}

	if(!nh.getParam("radar_position_bias_time_constant", biasTimeConstants(8))){
		successfulRead = false;
		biasTimeConstants(8) = 1;
	}

	if(!nh.getParam("radar_size_bias_time_constant", biasTimeConstants(9))){
		successfulRead = false;
		biasTimeConstants(9) = 1;
	}
	biasTimeConstants(10) = 1;
	biasTimeConstants(11) = 1;
	biasTimeConstants(12) = 1;
	Tb = Eigen::MatrixXd::Zero(nb,nb);
	for(int i = 0; i < nb; i++){
		Tb(i,i) = -1/biasTimeConstants(i);
	}

	return successfulRead;
}

void detectedObject::set_true_position(gpsPoint truePos){
	X(0) = truePos.longitude;
	X(1) = truePos.latitude;
}

void detectedObject::set_true_COG(double trueCOG){
	X(2) = trueCOG;
}

void detectedObject::set_true_SOG(double trueSOG){
	X(3) = trueSOG;
}

void detectedObject::set_true_radius(double trueCS){
	X(4) = trueCS;
}


void detectedObject::set_AIS_data(double SOG, double COG, double ROT, gpsPoint position){
	Za = Eigen::VectorXd(5);
	Za << position.longitude, position.latitude, COG, SOG, ROT;
	lastAISupdate = QTime::currentTime();
	AIS_ON = true;
}

void detectedObject::set_lidar_active(bool active){
	lidar_ON = active;
	if(active){ lastLidarUpdate = QTime::currentTime(); }
}

void detectedObject::set_radar_active(bool active){
	radar_ON = active;
	if(active){ lastRadarUpdate = QTime::currentTime(); }
}

bool detectedObject::is_active(){
	return AIS_ON || radar_ON || lidar_ON;
}

void Kalman_project_ahead(	Eigen::VectorXd &X_bar,
							Eigen::VectorXd X_hat, 
							Eigen::VectorXd &f_hat,
							Eigen::MatrixXd &P_bar,
							Eigen::MatrixXd P, 
							Eigen::MatrixXd T_b, 
							double h, 
							Eigen::MatrixXd E, 
							Eigen::MatrixXd Q,
							bool fromAIS)
{
	Eigen::MatrixXd I_19 = Eigen::MatrixXd::Identity(19,19);
	double longDegsPrM = longitude_degs_pr_meter(X_hat(1));
	double latDegsPrM = latitude_degs_pr_meter();

	Eigen::VectorXd b(13);
	b <<  X_hat(6), X_hat(7), X_hat(8), X_hat(9), X_hat(10), X_hat(11), X_hat(12), X_hat(13), X_hat(14), X_hat(15), X_hat(16), X_hat(17), X_hat(18); 
	double psi = deg2rad( X_hat(2) );
	double u = X_hat(3);
	double rot = b(0);
	f_hat << u*sin(psi)*longDegsPrM, u*cos(psi)*latDegsPrM, rot, 0, b(1), 0, T_b*b;
	Eigen::MatrixXd J(19,19);
	J << 	0, 0, u*cos(psi)*longDegsPrM, sin(psi)*longDegsPrM, Eigen::MatrixXd::Zero(1,15),
			0, 0, -u*sin(psi)*latDegsPrM, cos(psi)*latDegsPrM, Eigen::MatrixXd::Zero(1,15),
			Eigen::MatrixXd::Zero(1,6), 1, Eigen::MatrixXd::Zero(1,12), 
			Eigen::MatrixXd::Zero(1,6), 0, Eigen::MatrixXd::Zero(1,12),
			Eigen::MatrixXd::Zero(1,7), 1, Eigen::MatrixXd::Zero(1,11),
			Eigen::MatrixXd::Zero(1,19),
			Eigen::MatrixXd::Zero(13,6),  T_b;

	Eigen::MatrixXd PHI = I_19 + h*J;

	Eigen::MatrixXd Gamma = h*E;
	X_bar = X_hat + h*f_hat;
	P_bar = PHI*P*PHI.transpose() + Gamma*Q*Gamma;
}

void print_X(Eigen::VectorXd X, string s){
	qDebug() << "-----------------------------------";
	cout << "\n" << s << "\n";
	qDebug() << "Pos:\t\t" << X(0) << "," << X(1);
	qDebug() << "Heading:\t" << X(2);
	qDebug() << "Speed:\t\t" << X(3);
	qDebug() << "ROT:\t\t" << X(4);
	qDebug() << "CS:\t\t" << X(5);
	qDebug() << "-----------------------------------";
}

Eigen::VectorXd detectedObject::KalmanFusion(){
	Eigen::MatrixXd I_19 = Eigen::MatrixXd::Identity(19,19);

	// AIS
	if( Za != Za_prev && AIS_ON){
		Eigen::MatrixXd K = Pa_bar*Ha.transpose()*(Ha*Pa_bar*Ha.transpose() + Ra ).inverse();
		Xa_hat = Xa_bar + K*(Za - Ha*Xa_bar);
		Pa = (I_19 -K*Ha)*Pa_bar*((I_19 - K*Ha).transpose()) + K*Ra*K.transpose();

		Kalman_project_ahead(Xa_bar, Xa_hat, fa_hat, Pa_bar, Pa, Tb, ha, E, Q, true);
		Za_prev = Za;

	}
	else if(AIS_ON){
		Xa_hat = Xa_hat + hr*fa_hat;
	}

	// Radar
	if (radar_ON){
		Eigen::MatrixXd Kr = Pr_bar*Hr.transpose()*(Hr*Pr_bar*Hr.transpose() + Rr).inverse();
		Xr_hat = Xr_bar + Kr*(Zr - Hr*Xr_bar);
		Pr = (I_19 - Kr*Hr)*Pr_bar*((I_19 - Kr*Hr).transpose()) + Kr*Rr*Kr.transpose();
		Eigen::VectorXd fr_hat(19);

		Kalman_project_ahead(Xr_bar, Xr_hat, fr_hat, Pr_bar, Pr, Tb, hr, E, Q, false);
	}

	// lidar
	if (lidar_ON)
	{
		Eigen::MatrixXd Kl = Pl_bar*Hl.transpose()*(Hl*Pl_bar*Hl.transpose() + Rl).inverse();
		Xl_hat = Xl_bar + Kl*(Zl - Hl*Xl_bar);
		Pl = (I_19 - Kl*Hl)*Pl_bar*((I_19 - Kl*Hl).transpose()) + Kl*Rl*Kl.transpose();
		Eigen::VectorXd fl_hat(19);

		Kalman_project_ahead(Xl_bar, Xl_hat, fl_hat, Pl_bar, Pl, Tb, hl, E, Q, false);
	}


	// Simple Fusion Algorithm:
	Eigen::VectorXd X_hat(nk);
	Eigen::MatrixXd Pa_w = (Pa*pow(ha,-2)).inverse();
	Eigen::MatrixXd Pr_w = (Pr*pow(hr,-2)).inverse();
	Eigen::MatrixXd Pl_w = (Pl*pow(hl,-2)).inverse();

	if( AIS_ON && !radar_ON && !lidar_ON ){
		// qDebug() << "Using AIS only.";
		X_hat = Xa_hat;
		//cout << "Za:\n" << Za << endl;
	}
	else if( !AIS_ON && radar_ON && !lidar_ON ){
		// qDebug() << "Using RADAR only.";
		X_hat = Xr_hat;
	}
	else if( !AIS_ON && !radar_ON && lidar_ON){
		// qDebug() << "Using LiDAR only.";
		X_hat = Xr_hat;
	}
	else if(AIS_ON && radar_ON && lidar_ON){
		// qDebug() << "Using SENSOR FUSION: AIS, Radar, LiDAR.";
		X_hat = (Pa_w + Pr_w + Pl_w).inverse()*(Pa_w*Xa_hat + Pr_w*Xr_hat + Pl_w*Xl_hat);
	}
	else if(AIS_ON && radar_ON){
		// qDebug() << "Using SENSOR FUSION: AIS, Radar.";
		X_hat = (Pa_w + Pr_w).inverse()*(Pa_w*Xa_hat + Pr_w*Xr_hat);
	}
	else if(AIS_ON && lidar_ON){
		// qDebug() << "Using SENSOR FUSION: AIS, LiDAR.";
		X_hat = (Pa_w + Pl_w).inverse()*(Pa_w*Xa_hat + Pl_w*Xl_hat);
	}
	else if(radar_ON && lidar_ON){
		// qDebug() << "Using SENSOR FUSION: Radar, LiDAR.";
		X_hat = (Pr_w + Pl_w).inverse()*(Pr_w*Xr_hat + Pl_w*Xl_hat);
	}

	double e_longr = (Xr_hat(0) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_latr = (Xr_hat(1) - X(1))/latitude_degs_pr_meter();
	double e_pr = sqrt(pow(e_longr,2) + pow(e_latr,2));
	double e_longl = (Xl_hat(0) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_latl = (Xl_hat(1) - X(1))/latitude_degs_pr_meter();
	double e_pl = sqrt(pow(e_longl,2) + pow(e_latl,2));

	double e_longa = (Xa_hat(0) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_lata = (Xa_hat(1) - X(1))/latitude_degs_pr_meter();
	double e_pa = sqrt(pow(e_longa,2) + pow(e_lata,2));
	double e_hr = Za(1) - Xr_hat(1);
	double e_ha = Za(1) - Xa_hat(1);
	double e_long = (X_hat(0) - X(0))/longitude_degs_pr_meter(Zr(1));
	double e_lat = (X_hat(1) - X(1))/latitude_degs_pr_meter();
	double e_p = sqrt(pow(e_long,2) + pow(e_lat,2));
	double e_h = Za(1) - X_hat(1);
	double e_r = sqrt(pow((Zr(0) - X(0))/longitude_degs_pr_meter(Zr(1)) ,2) + pow((Zr(1) - X(1))/latitude_degs_pr_meter() ,2));
	double e_a = sqrt(pow((Za(0) - X(0))/longitude_degs_pr_meter(Zr(1)) ,2) + pow((Za(1) - X(1))/latitude_degs_pr_meter() ,2));
/*
	cout << "\nFusion position  error: " << e_p << endl;
	cout << "\nAIS position  error: " << e_pa << endl;
	cout << "\nRadar position  error: " << e_pr << endl;
	cout << "\nLiDAR position  error: " << e_pl << endl;
*/

	Eigen::VectorXd X_hat_short = Eigen::VectorXd::Zero(n);
	double positionErrorTreshold = 50;
	if (abs(e_p) > positionErrorTreshold )
	{
		if (AIS_ON){
			X_hat_short << Za(0), Za(1), Za(2), Za(3), pow(200,2);
		}
		else if(radar_ON){
			X_hat_short << Zr(0), Zr(1), 0, 0, Zr(2);
		}
	}
	else{
		X_hat_short << X_hat(0), X_hat(1), X_hat(2), X_hat(3), X_hat(5);
	}
	Eigen::VectorXd Xr_hat_short = Eigen::VectorXd::Zero(n);
	Eigen::VectorXd Xa_hat_short = Eigen::VectorXd::Zero(n);
	Xr_hat_short << Xr_hat(0), Xr_hat(1), fmod(Xr_hat(2), 360.0), Xr_hat(3), Xr_hat(5);
	Xa_hat_short << Xa_hat(0), Xa_hat(1), fmod(Xa_hat(2), 360.0), Xa_hat(3), Xa_hat(5);

	//print_X(Xr_hat,"Xr_hat");

	return X_hat_short;
}
















