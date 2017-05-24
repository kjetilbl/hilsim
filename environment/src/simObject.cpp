#include "simObject.h"

#include <algorithm> 

#include <QDebug>

int fixedObstacle::IDiterator = 0;
int aisUser::IDiterator = 0;

simObject::simObject( const simObject& other )
{
	this->nh = other.nh;
	this->cmdSub = nh->subscribe("/simObject/command", 1000, &simObject::command_parser, this);
	this->posUpdatePub = nh->advertise<environment::obstacleUpdate>("/simObject/position", 1000);

	this->ID = other.ID;
	this->eta = other.eta;
}

simObject::simObject(ros::NodeHandle *n, string obstID, gpsPoint3DOF eta0, double Size, QThread *parent) : QThread(parent)
{
	this->nh = n;
	this->cmdSub = nh->subscribe("/simObject/command", 1000, &simObject::command_parser, this);
	this->posUpdatePub = nh->advertise<environment::obstacleUpdate>("/simObject/position", 1000);

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
	this->posReportTimer->start(50);
}

void simObject::run()
{
	QThread::exec();
}

void simObject::publish_position_report()
{
	environment::obstacleUpdate posUpdate = make_position_update_msg();
	//qDebug() << "Publishing pos report from: " << this->ID.c_str();
	this->posUpdatePub.publish(posUpdate);
}

void simObject::command_parser(const environment::obstacleCmd::ConstPtr& cmd)
{
	// Forbeholdt terminate-kommando
}


environment::obstacleUpdate simObject::make_position_update_msg()
{
	lock_guard<mutex> lock(m);

	environment::obstacleUpdate posUpdate;
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
	ros::AsyncSpinner spinner(1);
	//spinner.start();
	QThread::exec();
}


aisUser::aisUser( ros::NodeHandle *n, gpsPoint3DOF eta0, double Size, QThread *parent ) 
				: simObject( n, "AIS_user_"+to_string(this->IDiterator), eta0, Size, parent )
{
	this->set_MMSI(IDiterator++);
	this->AISpub = n->advertise<simulator_messages::AIS>("sensors/ais", 1000);
}


void aisUser::broadcast_AIS_msg()
{
	gpsPoint3DOF currentEta = this->get_eta();
	navData nd( this->get_MMSI(), currentEta.longitude, currentEta.latitude, this->get_SOG(), currentEta.heading );
	nd.set_nav_status( this->get_status() );
	nd.set_ROT( this->get_ROT() );
	nd.set_position_accuracy( this->get_pos_accuracy() );
	nd.set_COG( currentEta.heading );
	string rawAISdata = nd.get_AIS_class_A_position_report();

	simulator_messages::AIS newAISmsg = nd.get_AIS_ros_msg();
	this->AISpub.publish(newAISmsg);
	//ros::spinOnce();
	/*
	qDebug() << "---------------------Publish" << this->ID.c_str() << "AIS message--------------------- ";
	nd.print_data();
	qDebug() << "-----------------------------------------------------------------------------";
	*/
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
	this->AIStimer->start(intervalMs);
}



ship::ship( ros::NodeHandle *n, gpsPoint3DOF eta0, double Size, QThread *parent ) 
				: aisUser( n, eta0, Size, parent )
{
	this->objectDescriptor = "ship";
	this->set_status(UNDERWAY_USING_ENGINE);
	this->set_ROT(0);
	this->set_SOG(5);
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

	ros::AsyncSpinner spinner(1);
	//spinner.start();

	QThread::exec();
}

gpsPoint3DOF ship::calculate_next_eta(){
	gpsPoint3DOF Eta = this->get_eta();
	gpsPoint nextWaypoint; 

	if(!waypoints.empty()){
		nextWaypoint = waypoints.front();
		if(distance_m(Eta, nextWaypoint) < 3){
			waypoints.erase(waypoints.begin());
		}
	}
	else{
		this->set_SOG(0);
		return Eta;
	}

	double Speed = this->get_SOG(); 					// ms
	double dt = ((double)this->stepIntervalMs)/1000; 	// s
	gpsPoint3DOF nextEta;

	// Kinematic model, using Eulor method
	double dE = Speed*sin(deg2rad(Eta.heading))*dt; 	// East
	double dN = Speed*cos(deg2rad(Eta.heading))*dt; 	// North
	nextEta.longitude = Eta.longitude + dE*longitude_degs_pr_meter(Eta.latitude);
	nextEta.latitude = Eta.latitude + dN*latitude_degs_pr_meter();

	// Calculate bearing to next waypoint
	nextEta.heading = compass_bearing(Eta, nextWaypoint);

	return nextEta;
}

void ship::step()
{
	gpsPoint3DOF nextEta = this->calculate_next_eta();
	this->set_eta(nextEta);
}
