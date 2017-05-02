#include "obstacleControl.h"

obstacleHandler::obstacleHandler(ros::NodeHandle nh, QThread *parent ) : QThread(parent) {
	this->n = nh;
	this->cmdSub = nh.subscribe("obstacleCommandTopic", 1000, &obstacleHandler::command_parser, this);
	get_origin_from_sim_params(nh);
}

obstacleHandler::~obstacleHandler(){
	testShip->quit();
	testShip->wait();
	delete testShip;

	for(auto const& simObjectPtr: agents){
		simObjectPtr->quit();
		simObjectPtr->wait();
		delete simObjectPtr;
	}
	if( simObjectsThread != NULL ){
		simObjectsThread->quit();
		simObjectsThread->wait();
		delete simObjectsThread;
	}
}

void obstacleHandler::command_parser(const environment::obstacleCmd::ConstPtr& cmd)
{
	static int obstIterator = 1;
	if(cmd->cmdSpecifier == "spawn")
	{
		// string ID = "fixed_obstacle_" + to_string(obstIterator++);
		if ( simObjectsThread == NULL )
		{
			simObjectsThread = new QThread();
			simObjectsThread->start();
		}

		double longitude = cmd->x;
		double latitude = cmd->y;
		double psi = cmd->psi;
		ROS_INFO("%s %s [%f %f %f]", cmd->cmdSpecifier.c_str(), cmd->receiverID.c_str(), longitude, latitude, psi);

		ship* newObstacle = new ship(n, obstIterator++, longitude, latitude, psi);
		newObstacle->moveToThread( simObjectsThread );
		QObject::connect(simObjectsThread, SIGNAL(finished()), newObstacle, SLOT(deleteLater()) );
	    newObstacle->start();

		agents.push_back( newObstacle );

	}
}

void obstacleHandler::run()
{
	simObjectsThread = new QThread();
	simObjectsThread->start();

	spawn_ships();

	ros::AsyncSpinner spinner(1);
	spinner.start();
	QThread::exec();
}

void obstacleHandler::spawn_ships(){
	for(int i =  0; i < 1; i++){
		ship* newShip = new ship(n, i, mapOrigin.longitude, mapOrigin.latitude, 45*i);
		newShip->moveToThread(simObjectsThread);
		QObject::connect(simObjectsThread, SIGNAL(finished()), newShip, SLOT(deleteLater()) );
		newShip->start();
		agents.push_back( newShip );
	}
}

void obstacleHandler::get_origin_from_sim_params(ros::NodeHandle nh){
	nh.getParam("start_longitude", mapOrigin.longitude);
	nh.getParam("start_latitude", mapOrigin.latitude);
}

simObject::simObject( const simObject& other )
{
	this->n = other.n;
	this->cmdSub = n.subscribe("obstacleCommandTopic", 1000, &simObject::command_parser, this);
	this->posUpdatePub = n.advertise<environment::obstacleUpdate>("obstUpdateTopic", 1000);

	this->ID = other.ID;
	this->longitude = other.longitude;
	this->latitude = other.latitude;
	this->psi = other.psi;
}

simObject::simObject(ros::NodeHandle nh, string obstID, double Longitude, double Latitude, double Psi, QThread *parent) : QThread(parent)
{
	this->n = nh;
	this->cmdSub = n.subscribe("obstacleCommandTopic", 1000, &simObject::command_parser, this);
	this->posUpdatePub = n.advertise<environment::obstacleUpdate>("obstUpdateTopic", 1000);

	this->ID = obstID;
	this->longitude = Longitude;
	this->latitude = Latitude;
	this->psi = Psi;

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
	posUpdate.longitude = this->longitude;
	posUpdate.latitude = this->latitude;
	posUpdate.heading = this->psi;
	return posUpdate;
}

void simObject::set_position(double Longitude, double Latitude, double Psi)
{
	lock_guard<mutex> lock(m);
	this->longitude = Longitude;
	this->latitude = Latitude;
	this->psi  = Psi;
}

double simObject::get_longitude()
{
	lock_guard<mutex> lock(m);
	return this->longitude;
}

double simObject::get_latitude()
{
	lock_guard<mutex> lock(m);
	return this->latitude;
}

double simObject::get_heading()
{
	lock_guard<mutex> lock(m);
	return this->psi;
}


fixedObstacle::fixedObstacle( ros::NodeHandle nh, string obstID, double Longitude, double Latitude, double Psi, QThread *parent ) 
							: simObject( nh, obstID, Longitude, Latitude, Psi, parent )
{
	this->objectDescriptor = "fixed_obstacle";
}


void fixedObstacle::move()
{
	// dont move...
}

activeSimObject::activeSimObject( ros::NodeHandle nh, uint32_t mmsiNumber, double X, double Y, double Psi, QThread *parent ) 
				: simObject( nh, "active_object_"+to_string(mmsiNumber), X, Y, Psi, parent )
{
	
}


void activeSimObject::broadcast_AIS_msg()
{
	navData nd( this->get_MMSI(), this->get_longitude(), this->get_latitude(), this->get_SOG() );
	nd.set_nav_status( this->get_status() );
	nd.set_ROT( this->get_ROT() );
	nd.set_position_accuracy( this->get_pos_accuracy() );
	nd.set_COG( this->get_heading() );

	string AISmsg = nd.get_AIS_class_A_position_report(); // Should be broadcast
	qDebug() << "---------------------Publish" << this->ID.c_str() << "AIS message--------------------- ";
	nd.print_data();
	qDebug() << "-----------------------------------------------------------------------------";
}

void activeSimObject::set_MMSI(uint32_t ID){
	lock_guard<mutex> lock(activeObjMutex);
	this->MMSI = ID;
}

uint32_t activeSimObject::get_MMSI(){
	lock_guard<mutex> lock(activeObjMutex);
	return this->MMSI;
}

void activeSimObject::set_status(navStatus newStatus){
	lock_guard<mutex> lock(activeObjMutex);
	this->status = newStatus;
}

navStatus activeSimObject::get_status(){
	lock_guard<mutex> lock(activeObjMutex);
	return this->status;
}

void activeSimObject::set_ROT(double rot){
	lock_guard<mutex> lock(activeObjMutex);
	this->ROT = rot;
}


double activeSimObject::get_ROT(){
	lock_guard<mutex> lock(activeObjMutex);
	return this->ROT;
}

void activeSimObject::set_SOG(double sog){
	lock_guard<mutex> lock(activeObjMutex);
	this->SOG = sog;
}

double activeSimObject::get_SOG(){
	lock_guard<mutex> lock(activeObjMutex);
	return this->SOG;
}

void activeSimObject::set_pos_accuracy(posAccuracy accuracy){
	lock_guard<mutex> lock(activeObjMutex);
	this->positionAccuracy = accuracy;
}

posAccuracy activeSimObject::get_pos_accuracy()
{
	lock_guard<mutex> lock(activeObjMutex);
	return this->positionAccuracy;
}

void activeSimObject::initiate_AIS_broadcast(uint16_t intervalMs){
	if( AIStimer == NULL ){
		AIStimer = new QTimer(0);
		QObject::connect( AIStimer, SIGNAL(timeout()), this, SLOT(broadcast_AIS_msg()) );
	}
	this->AIStimer->start(intervalMs);
}



ship::ship( ros::NodeHandle nh, uint32_t mmsiNumber, double Longitude, double Latitude, double Psi, QThread *parent ) 
				: activeSimObject( nh, mmsiNumber, Longitude, Latitude, Psi, parent )
{
	this->objectDescriptor = "ship";
	this->set_MMSI(mmsiNumber);
	this->set_status(UNDERWAY_USING_ENGINE);
	this->set_ROT(0);
	this->set_SOG(2);
	this->set_pos_accuracy(HIGH);
}


void ship::move()
{
	double currentLongitude = this->get_longitude();
	double currentLatitude = this->get_latitude();
	double currentHeading = this->get_heading();
	double speed = this->get_SOG(); 					// ms
	double dt = ((double)this->moveIntervalMs)/1000; 	// s


	double dE = speed*sin(deg2rad(currentHeading))*dt; // East
	double dN = speed*cos(deg2rad(currentHeading))*dt; // North

	double nextLat = currentLatitude + dN*latitude_degs_pr_meter();
	double nextLong = currentLongitude + dE*longitude_degs_pr_meter(currentLatitude);

	this->set_position( nextLong, nextLat, currentHeading );
}

void ship::run()
{
	this->moveIntervalMs = 100; // ms
	this->moveTimer = new QTimer(0);
	QObject::connect( moveTimer, SIGNAL(timeout()), this, SLOT(move()) );
	this->moveTimer->start(moveIntervalMs);

	this->initiate_AIS_broadcast(2000);
	this->initiate_pos_report_broadcast();

	QThread::exec();
}