#include "simObject.h"

#include <algorithm> 

#include <QDebug>


simObject::simObject( const simObject& other )
{
	this->n = other.n;
	this->cmdSub = n.subscribe("obstacleCommandTopic", 1000, &simObject::command_parser, this);
	this->posUpdatePub = n.advertise<environment::obstacleUpdate>("obstUpdateTopic", 1000);

	this->ID = other.ID;
	this->eta = other.eta;
}

simObject::simObject(ros::NodeHandle nh, string obstID, gpsPoint3DOF eta0, QThread *parent) : QThread(parent)
{
	this->n = nh;
	this->cmdSub = n.subscribe("obstacleCommandTopic", 1000, &simObject::command_parser, this);
	this->posUpdatePub = n.advertise<environment::obstacleUpdate>("obstUpdateTopic", 1000);

	this->ID = obstID;
	this->eta = eta0;

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
	this->posUpdatePub.publish(posUpdate);
	ros::spinOnce();
}

void simObject::command_parser(const environment::obstacleCmd::ConstPtr& cmd)
{
	ROS_INFO("%s received a new command!", this->ID.c_str()); // Forbeholdt terminate-kommando
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


fixedObstacle::fixedObstacle( ros::NodeHandle nh, string obstID, gpsPoint3DOF eta0, QThread *parent ) 
							: simObject( nh, obstID, eta0, parent )
{
	this->objectDescriptor = "fixed_obstacle";
}


aisUser::aisUser( ros::NodeHandle nh, uint32_t mmsiNumber, gpsPoint3DOF eta0, QThread *parent ) 
				: simObject( nh, "AIS_user_"+to_string(mmsiNumber), eta0, parent )
{

}


void aisUser::broadcast_AIS_msg()
{
	gpsPoint3DOF currentEta = this->get_eta();
	navData nd( this->get_MMSI(), currentEta.longitude, currentEta.latitude, this->get_SOG() );
	nd.set_nav_status( this->get_status() );
	nd.set_ROT( this->get_ROT() );
	nd.set_position_accuracy( this->get_pos_accuracy() );
	nd.set_COG( currentEta.heading );

	string AISmsg = nd.get_AIS_class_A_position_report(); // Should be broadcast
	qDebug() << "---------------------Publish" << this->ID.c_str() << "AIS message--------------------- ";
	nd.print_data();
	qDebug() << "-----------------------------------------------------------------------------";
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



ship::ship( ros::NodeHandle nh, uint32_t mmsiNumber, gpsPoint3DOF eta0, QThread *parent ) 
				: aisUser( nh, mmsiNumber, eta0, parent )
{
	this->objectDescriptor = "ship";
	this->set_MMSI(mmsiNumber);
	this->set_status(UNDERWAY_USING_ENGINE);
	this->set_ROT(0);
	this->set_SOG(2);
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

gpsPoint3DOF ship::calculate_next_eta(){
	gpsPoint3DOF Eta = this->get_eta();
	gpsPoint nextWaypoint; 

	if(!waypoints.empty()){
		nextWaypoint = waypoints.front();
		if(distance(Eta, nextWaypoint) < 3){
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

	//TODO: calculate next heading according to next waypoint
	nextEta.heading = Eta.heading;



	return nextEta;
}

void ship::step()
{
	gpsPoint3DOF nextEta = this->calculate_next_eta();
	this->set_eta(nextEta);
}
