#include "simObject.h"

#include <algorithm> 

#include <QDebug>

int fixedObstacle::IDiterator = 0;
int aisUser::IDiterator = 0;

simObject::simObject( const simObject& other )
{
	this->nh = other.nh;
	//this->cmdSub = nh->subscribe("/simObject/command", 1000, &simObject::command_parser, this);
	this->posUpdatePub = nh->advertise<simulator_messages::obstacleUpdate>("/simObject/position", 1000);

	this->ID = other.ID;
	this->eta = other.eta;
}

simObject::simObject(ros::NodeHandle *n, string obstID, gpsPoint3DOF eta0, double Size, QThread *parent) : QThread(parent)
{
	this->nh = n;
	//this->cmdSub = nh->subscribe("/simObject/command", 1000, &simObject::command_parser, this);
	this->posUpdatePub = nh->advertise<simulator_messages::obstacleUpdate>("/simObject/position", 1000);

	this->ID = obstID;
	this->eta = eta0;
	this->size = Size;

}

simObject::~simObject(){
	delete posReportTimer;
}

void simObject::initiate_pos_report_broadcast()
{
	this->posReportTimer = new QTimer(0);
	QObject::connect( posReportTimer, SIGNAL(timeout()), this, SLOT(publish_position_report()) );
	this->posReportTimer->start(200);
}

void simObject::run()
{
	QThread::exec();
}

void simObject::publish_position_report()
{
	simulator_messages::obstacleUpdate posUpdate = make_position_update_msg();
	//qDebug() << "Publishing pos report from: " << this->ID.c_str();
	this->posUpdatePub.publish(posUpdate);
}

void simObject::command_parser(const simulator_messages::obstacleCmd::ConstPtr& cmd)
{
	// Forbeholdt terminate-kommando
}


simulator_messages::obstacleUpdate simObject::make_position_update_msg()
{
	lock_guard<mutex> lock(m);

	simulator_messages::obstacleUpdate posUpdate;
	posUpdate.msgDescriptor = "position_update";
	posUpdate.objectDescriptor = this->objectDescriptor;
	posUpdate.objectID = this->ID;
	posUpdate.longitude = this->eta.longitude;
	posUpdate.latitude = this->eta.latitude;
	posUpdate.heading = this->eta.heading;
	posUpdate.size = this->size;
	return posUpdate;
}

void simObject::set_eta(gpsPoint3DOF newEta)
{
	lock_guard<mutex> lock(m);
	this->eta = newEta;
}

gpsPoint3DOF simObject::get_eta()
{
	lock_guard<mutex> lock(m);
	return this->eta;
}


fixedObstacle::fixedObstacle( ros::NodeHandle *n, gpsPoint3DOF eta0, double Size, QThread *parent ) 
							: simObject( n, "fixed_obstacle_"+to_string(this->IDiterator++), eta0, Size, parent )
{
	this->objectDescriptor = "fixed_obstacle";
}

void fixedObstacle::run(){
	this->initiate_pos_report_broadcast();
	QThread::exec();
}


aisUser::aisUser( ros::NodeHandle *n, gpsPoint3DOF eta0, double Size, QThread *parent ) 
				: simObject( n, "AIS_user_"+to_string(this->IDiterator), eta0, Size, parent )
{
	int N = 6;
	b = Eigen::VectorXd::Zero(N);
	Td = Eigen::MatrixXd::Zero(N,N);
	this->set_MMSI(IDiterator++);
	this->AISpub = n->advertise<simulator_messages::AIS>("sensors/ais", 1000);
}


void aisUser::broadcast_AIS_msg()
{	

	Eigen::VectorXd Xm = get_estimated_nav_parameters();

	navData nd( this->get_MMSI(), Xm(0), Xm(1), Xm(5), Xm(3) );
	nd.set_COG( Xm(2) );
	nd.set_track( Xm(2) );
	nd.set_ROT( Xm(4) );
	nd.set_nav_status( this->get_status() );
	nd.set_position_accuracy( this->get_pos_accuracy() );
	string rawAISdata = nd.get_AIS_class_A_position_report();

	simulator_messages::AIS newAISmsg = nd.get_AIS_ros_msg();
	if (AISenabled)
	{
		this->AISpub.publish(newAISmsg);
	}
}

void aisUser::set_MMSI(uint32_t ID){
	lock_guard<mutex> lock(activeObjMutex);
	this->MMSI = ID;
}

uint32_t aisUser::get_MMSI(){
	lock_guard<mutex> lock(activeObjMutex);
	return this->MMSI;
}

void aisUser::set_status(navStatus newStatus){
	lock_guard<mutex> lock(activeObjMutex);
	this->status = newStatus;
}

navStatus aisUser::get_status(){
	lock_guard<mutex> lock(activeObjMutex);
	return this->status;
}

void aisUser::set_ROT(double rot){
	lock_guard<mutex> lock(activeObjMutex);
	this->ROT = rot;
}


double aisUser::get_ROT(){
	lock_guard<mutex> lock(activeObjMutex);
	return this->ROT;
}

void aisUser::set_SOG(double sog){
	lock_guard<mutex> lock(activeObjMutex);
	this->SOG = sog;
}

double aisUser::get_SOG(){
	lock_guard<mutex> lock(activeObjMutex);
	return this->SOG;
}

void aisUser::set_pos_accuracy(posAccuracy accuracy){
	lock_guard<mutex> lock(activeObjMutex);
	this->positionAccuracy = accuracy;
}

posAccuracy aisUser::get_pos_accuracy()
{
	lock_guard<mutex> lock(activeObjMutex);
	return this->positionAccuracy;
}

void aisUser::initiate_AIS_broadcast(uint16_t intervalMs){
	if( AIStimer == NULL ){
		AIStimer = new QTimer(0);
		QObject::connect( AIStimer, SIGNAL(timeout()), this, SLOT(broadcast_AIS_msg()) );
	}
	this->AISinterval = intervalMs;
	this->AIStimer->start(intervalMs);
}

bool aisUser::read_AIS_config(){
	bool successfulRead = true;
	int n = 6;
	biasSigmas = Eigen::VectorXd(n);
	double positionBiasSigma = 0;
	if(!nh->getParam("AIS_position_bias_sigma", positionBiasSigma)){
		successfulRead = false;
	}
	biasSigmas(0) = positionBiasSigma*longitude_degs_pr_meter(this->get_eta().latitude);
	biasSigmas(1) = positionBiasSigma*latitude_degs_pr_meter();

	if(!nh->getParam("AIS_COG_bias_sigma", biasSigmas(2))){
		successfulRead = false;
		biasSigmas(2) = 0;
	}

	if(!nh->getParam("AIS_track_bias_sigma", biasSigmas(3))){
		successfulRead = false;
		biasSigmas(3) = 0;
	}

	if(!nh->getParam("AIS_ROT_bias_sigma", biasSigmas(4))){
		successfulRead = false;
		biasSigmas(4) = 0;
	}

	if(!nh->getParam("AIS_SOG_bias_sigma", biasSigmas(5))){
		successfulRead = false;
		biasSigmas(5) = 0;
	}


	measureSigmas = Eigen::VectorXd(n);
	double positionMeasureSigma = 0;
	if(!nh->getParam("AIS_position_measure_sigma", positionMeasureSigma)){
		successfulRead = false;
	}
	measureSigmas(0) = positionMeasureSigma*longitude_degs_pr_meter(this->get_eta().latitude);
	measureSigmas(1) = positionMeasureSigma*latitude_degs_pr_meter();

	if(!nh->getParam("AIS_COG_measure_sigma", measureSigmas(2))){
		successfulRead = false;
		measureSigmas(2) = 0;
	}

	if(!nh->getParam("AIS_track_measure_sigma", measureSigmas(3))){
		successfulRead = false;
		measureSigmas(3) = 0;
	}

	if(!nh->getParam("AIS_ROT_measure_sigma", measureSigmas(4))){
		successfulRead = false;
		measureSigmas(4) = 0;
	}

	if(!nh->getParam("AIS_SOG_measure_sigma", measureSigmas(5))){
		successfulRead = false;
		measureSigmas(5) = 0;
	}


	Eigen::VectorXd biasTimeConstants(n);
	double posBiasTimeConstant = 1;
	if(!nh->getParam("AIS_position_bias_time_constant", posBiasTimeConstant)){
		successfulRead = false;
	}
	biasTimeConstants(0) = posBiasTimeConstant;
	biasTimeConstants(1) = posBiasTimeConstant;

	if(!nh->getParam("AIS_COG_bias_time_constant", biasTimeConstants(2))){
		successfulRead = false;
		biasTimeConstants(2) = 0;
	}

	if(!nh->getParam("AIS_track_bias_time_constant", biasTimeConstants(3))){
		successfulRead = false;
		biasTimeConstants(3) = 0;
	}

	if(!nh->getParam("AIS_ROT_bias_time_constant", biasTimeConstants(4))){
		successfulRead = false;
		biasTimeConstants(3) = 0;
	}

	if(!nh->getParam("AIS_SOG_bias_time_constant", biasTimeConstants(5))){
		successfulRead = false;
		biasTimeConstants(3) = 0;
	}
	T = biasTimeConstants.asDiagonal();

	return successfulRead;
}

Eigen::VectorXd aisUser::get_estimated_nav_parameters(){
	static std::default_random_engine randomGenerator;
	static std::normal_distribution<double> gaussianWhiteNoise(0,1);

	if(!updatedParameters){
		if(!read_AIS_config()){
			qDebug() << "read_AIS_config() failed.";
			exit(EXIT_FAILURE);
		}
		updatedParameters = true;
	}
	if(firstTimeErrorCalc){
		lastErrorCalcTime = QTime::currentTime();
		firstTimeErrorCalc = false;
	}
	QTime now = QTime::currentTime();
	double dt = (double)(lastErrorCalcTime.msecsTo( now ))/1000;
	lastErrorCalcTime = now;

	gpsPoint3DOF currentEta = this->get_eta();
	int n = 6;
	Eigen::VectorXd Xm(n);
	Eigen::VectorXd X(n);
	X << 	currentEta.longitude,
			currentEta.latitude,
			currentEta.heading,
			currentEta.heading,
			this->get_ROT(),
			this->get_SOG();


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
	e(5) = max(-X(5), (double)e(5));

	// Set artificial measurements:
	Xm = X + e;

	return Xm;
}



ship::ship( ros::NodeHandle *n, 
			gpsPoint3DOF eta0, 
			double Size, 
			double SpeedInKnots,
			QThread *parent ) 
	: aisUser( n, eta0, Size, parent )
{
	this->objectDescriptor = "ship";
	this->set_status(UNDERWAY_USING_ENGINE);
	this->set_ROT(0);
	this->set_SOG(SpeedInKnots);
	this->set_pos_accuracy(HIGH);
}

void ship::add_waypoint(gpsPoint wp){
	waypoints.push_back(wp);
}

void ship::run()
{
	this->stepIntervalMs = 100; // ms
	this->stepTimer = new QTimer(0);
	QObject::connect( stepTimer, SIGNAL(timeout()), this, SLOT(step()) );
	this->stepTimer->start(stepIntervalMs);

	this->initiate_AIS_broadcast(2000);
	this->initiate_pos_report_broadcast();

	QThread::exec();
}


void ship::step()
{
	gpsPoint3DOF Eta = this->get_eta();
	gpsPoint nextWaypoint; 

	while(!waypoints.empty()){
		nextWaypoint = waypoints.front();
		if(distance_m(Eta, nextWaypoint) < 50){
			waypoints.erase(waypoints.begin());
		}
		else{
			break;
		}
	}
	if(waypoints.empty()){
		this->set_SOG(0);
		return;
	}

	double Speed = this->get_SOG();			// ms
	double dt = ((double)this->stepIntervalMs)/1000; 	// s
	gpsPoint3DOF nextEta;

	// Kinematic model, using Eulor method
	double dE = Speed*sin(deg2rad(Eta.heading))*dt; 	// East
	double dN = Speed*cos(deg2rad(Eta.heading))*dt; 	// North
	nextEta.longitude = Eta.longitude + dE*longitude_degs_pr_meter(Eta.latitude);
	nextEta.latitude = Eta.latitude + dN*latitude_degs_pr_meter();

	// Calculate next heading and turn rate
	double turnRateRef = 5; //degs/second
	double turnRate = this->get_ROT(); //degs/second
	double WPbearing = compass_bearing(Eta, nextWaypoint);
	if(abs(WPbearing - Eta.heading) < 2){
		nextEta.heading = WPbearing;
		turnRateRef = 0;
	}
	else if( WPbearing - Eta.heading > 180){
		turnRateRef *= -1;
	}
	else if( Eta.heading > WPbearing && Eta.heading - WPbearing < 180){
		turnRateRef *= -1;
	}
	turnRate = turnRateRef;
	nextEta.heading = fmod(Eta.heading + dt*turnRate, 360.0);
	while(nextEta.heading < 0){
		nextEta.heading += 360;
	}


	this->set_ROT(turnRate);
	this->set_eta(nextEta);
}
