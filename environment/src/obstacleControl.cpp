#include "obstacleControl.h"

obstacleHandler::obstacleHandler(ros::NodeHandle nh)
{
	n = nh;
	cmdSub = nh.subscribe("obstacleCommandTopic", 1000, &obstacleHandler::command_parser, this);
}


void obstacleHandler::command_parser(const environment::obstacleCmd::ConstPtr& cmd)
{
	static int obstIterator = 0;
	if(cmd->cmdSpecifier == "spawn")
	{
		// string ID = "fixed_obstacle_" + to_string(obstIterator++);
		static bool first = true;
		if ( first == true )
		{
			simObjectsThread = new QThread();
			simObjectsThread->start();
			first = false;
			qDebug() << "simObjectsThread started...";
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

	ros::spin();
}

simObject::simObject( const simObject& other )
{
	n = other.n;
	cmdSub = n.subscribe("obstacleCommandTopic", 1000, &simObject::command_parser, this);
	posUpdatePub = n.advertise<environment::obstacleUpdate>("obstUpdateTopic", 1000);

	ID = other.ID;
	longitude = other.longitude;
	latitude = other.latitude;
	psi = other.psi;
}

simObject::simObject(ros::NodeHandle nh, string obstID, double Longitude, double Latitude, double Psi, QThread *parent) : QThread(parent)
{
	n = nh;
	cmdSub = n.subscribe("obstacleCommandTopic", 1000, &simObject::command_parser, this);
	posUpdatePub = n.advertise<environment::obstacleUpdate>("obstUpdateTopic", 1000);

	ID = obstID;
	longitude = Longitude;
	latitude = Latitude;
	psi = Psi;

}

void simObject::initiate_pos_report_broadcast()
{
	posReportTimer = new QTimer(0);
	QObject::connect( posReportTimer, SIGNAL(timeout()), this, SLOT(publish_position_report()) );
	posReportTimer->start(50);
}

void simObject::run()
{
	//awrgargaergaergaregaerharehwgwagalgkjb|||\\------------
	/* TODO: posreporttimer starter ikke. Vil ha timer-basert run. Planen er å gjøre
	run() virtual, slik at subklasser kan lage egne run()s. Disse skal bruke timere for
	å sende AIS og position reports med ulike intervaller. 
	*/

	QThread::exec();
/*	
	ros::Rate loop_rate(24);
	while(true){
		move();

		loop_rate.sleep();

		lock_guard<mutex> lock(m);
		if(stop == true)
		{
			break;
		}
	}
	ROS_INFO("Will terminate simObject %s", ID.c_str());*/
}

void simObject::publish_position_report()
{
	environment::obstacleUpdate posUpdate = make_position_update_msg();
	posUpdatePub.publish(posUpdate);
	ros::spinOnce();
}

void simObject::command_parser(const environment::obstacleCmd::ConstPtr& cmd)
{
	ROS_INFO("%s received a new command!", ID.c_str()); // Forbeholdt terminate-kommando
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
	longitude = Longitude;
	latitude = Latitude;
	psi  = Psi;
}

double simObject::get_longitude()
{
	lock_guard<mutex> lock(m);
	return longitude;
}

double simObject::get_latitude()
{
	lock_guard<mutex> lock(m);
	return latitude;
}

double simObject::get_heading()
{
	lock_guard<mutex> lock(m);
	return psi;
}


fixedObstacle::fixedObstacle( ros::NodeHandle nh, string obstID, double Longitude, double Latitude, double Psi, QThread *parent ) 
							: simObject( nh, obstID, Longitude, Latitude, Psi, parent )
{
	objectDescriptor = "fixed_obstacle";
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
	nd.set_nav_status( get_status() );
	nd.set_ROT( get_ROT() );
	nd.set_position_accuracy( get_pos_accuracy() );
	nd.set_COG( this->get_heading() );

	string AISmsg = nd.get_AIS_class_A_position_report();
	qDebug() << "---------------------Publish" << this->ID.c_str() << "AIS message--------------------- ";
	nd.print_data();
	qDebug() << "-----------------------------------------------------------------------------";
}

void activeSimObject::set_MMSI(uint32_t ID){
	lock_guard<mutex> lock(activeObjMutex);
	MMSI = ID;
}

uint32_t activeSimObject::get_MMSI(){
	lock_guard<mutex> lock(activeObjMutex);
	return MMSI;
}

void activeSimObject::set_status(navStatus newStatus){
	lock_guard<mutex> lock(activeObjMutex);
	status = newStatus;
}

navStatus activeSimObject::get_status(){
	lock_guard<mutex> lock(activeObjMutex);
	return status;
}

void activeSimObject::set_ROT(double rot){
	lock_guard<mutex> lock(activeObjMutex);
	ROT = rot;
}


double activeSimObject::get_ROT(){
	lock_guard<mutex> lock(activeObjMutex);
	return ROT;
}

void activeSimObject::set_SOG(double sog){
	lock_guard<mutex> lock(activeObjMutex);
	SOG = sog;
}

double activeSimObject::get_SOG(){
	lock_guard<mutex> lock(activeObjMutex);
	return SOG;
}

void activeSimObject::set_pos_accuracy(posAccuracy accuracy){
	lock_guard<mutex> lock(activeObjMutex);
	positionAccuracy = accuracy;
}

posAccuracy activeSimObject::get_pos_accuracy()
{
	lock_guard<mutex> lock(activeObjMutex);
	return positionAccuracy;
}

void activeSimObject::initiate_AIS_broadcast(){
	AIStimer = new QTimer(0);
	QObject::connect( AIStimer, SIGNAL(timeout()), this, SLOT(broadcast_AIS_msg()) );
	AIStimer->start(2000);
	qDebug() << "Started AIS brodcast." << AIStimer->isActive();

}



ship::ship( ros::NodeHandle nh, uint32_t mmsiNumber, double Longitude, double Latitude, double Psi, QThread *parent ) 
				: activeSimObject( nh, mmsiNumber, Longitude, Latitude, Psi, parent )
{
	this->objectDescriptor = "ship";
	this->set_MMSI(mmsiNumber);
	this->set_status(UNDERWAY_USING_ENGINE);
	this->set_ROT(0);
	this->set_SOG(5);
	this->set_pos_accuracy(HIGH);
}


void ship::move()
{
	double currentLongitude = this->get_longitude();
	double currentLatitude = this->get_latitude();
	double currentHeading = this->get_heading();
	double speed = this->get_SOG(); 				// ms
	double dt = (double)this->moveIntervalMs/1000; 	// s

	double nextLat = currentLatitude + speed*cos(currentHeading)*dt*latitude_degs_pr_meter();
	double nextLong = currentLongitude + speed*sin(currentHeading)*dt*longitude_degs_pr_meter(currentLatitude);

	qDebug() << "Next:" << speed << dt << nextLong << nextLat;

	set_position( nextLong, nextLat, currentHeading );
}

void ship::run()
{
	this->moveIntervalMs = 100; // ms
	this->moveTimer = new QTimer(0);
	QObject::connect( moveTimer, SIGNAL(timeout()), this, SLOT(move()) );
	moveTimer->start(moveIntervalMs);

	this->initiate_AIS_broadcast();
	this->initiate_pos_report_broadcast();

	QThread::exec();
}