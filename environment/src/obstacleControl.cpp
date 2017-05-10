#include "obstacleControl.h"



obstacleHandler::obstacleHandler(ros::NodeHandle nh, QThread *parent ) : QThread(parent) {
	this->n = nh;
	this->cmdSub = nh.subscribe("/simObject/command", 1000, &obstacleHandler::command_parser, this);
	get_origin_from_sim_params(nh);
}

obstacleHandler::~obstacleHandler(){

	for(auto const& simObjectPtr: simObjects){
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
	static int obstIterator = 2;
	if(cmd->cmdSpecifier == "spawn")
	{
		ROS_INFO("%s %s [%f %f %f]", cmd->cmdSpecifier.c_str(), cmd->receiverID.c_str(), cmd->x, cmd->y, cmd->psi);

		gpsPoint3DOF eta0{cmd->x, cmd->y, cmd->psi};
		spawn_fixed_obstacle(eta0);
	}
}

void obstacleHandler::run()
{
	simObjectsThread = new QThread();
	simObjectsThread->start();

	spawn_obstacles();

	ros::AsyncSpinner spinner(1);
	spinner.start();
	QThread::exec();
}

void obstacleHandler::spawn_obstacles(){
	// Spawn ships
	double radius = 450;
	gpsPoint3DOF eta0;
	eta0.latitude = mapOrigin.latitude;
	eta0.longitude = mapOrigin.longitude+radius*longitude_degs_pr_meter(eta0.latitude);
	ship* ship1 = new ship(n, eta0);
	for(int i = 1; i < 128; i++){
		gpsPoint newWaypoint;
		newWaypoint.longitude = mapOrigin.longitude + radius*cos(M_PI/4*i)*longitude_degs_pr_meter(eta0.latitude); //m
		newWaypoint.latitude = mapOrigin.latitude + radius*sin(M_PI/4*i)*latitude_degs_pr_meter(); //m
		ship1->add_waypoint(newWaypoint);
	}
	ship1->moveToThread(simObjectsThread);
	QObject::connect(simObjectsThread, SIGNAL(finished()), ship1, SLOT(deleteLater()) );
	ship1->start();
	simObjects.push_back( ship1 );


	radius = radius*0.8;
	eta0.latitude = mapOrigin.latitude;
	eta0.longitude = mapOrigin.longitude - radius*longitude_degs_pr_meter(eta0.latitude);
	ship* ship2 = new ship(n, eta0);
	for(int i = 1; i < 128; i++){
		gpsPoint newWaypoint;
		newWaypoint.longitude = mapOrigin.longitude + radius*cos(-M_PI/4*i + M_PI)*longitude_degs_pr_meter(eta0.latitude); //m
		newWaypoint.latitude = mapOrigin.latitude + radius*sin(-M_PI/4*i + M_PI)*latitude_degs_pr_meter();
		ship2->add_waypoint(newWaypoint);
	}
	ship2->moveToThread(simObjectsThread);
	QObject::connect(simObjectsThread, SIGNAL(finished()), ship2, SLOT(deleteLater()) );
	ship2->start();
	simObjects.push_back( ship2 );
	

	// Spawn obstacles
	for(int i = 0; i < 25; i++){
		eta0.longitude = mapOrigin.longitude + (rand()%1000 - 500)*longitude_degs_pr_meter(mapOrigin.latitude);
		eta0.latitude = mapOrigin.latitude + (rand()%1000 - 500)*latitude_degs_pr_meter();
		spawn_fixed_obstacle(eta0);
	}

}


void obstacleHandler::spawn_fixed_obstacle(gpsPoint3DOF eta){
	if ( simObjectsThread == NULL )
	{
		simObjectsThread = new QThread();
		simObjectsThread->start();
	}
	fixedObstacle* newObstacle = new fixedObstacle(this->n, eta);
	newObstacle->moveToThread( this->simObjectsThread );
	QObject::connect(this->simObjectsThread, SIGNAL(finished()), newObstacle, SLOT(deleteLater()) );
	newObstacle->start();
	this->simObjects.push_back( newObstacle );
}


void obstacleHandler::get_origin_from_sim_params(ros::NodeHandle nh){
	nh.getParam("start_longitude", mapOrigin.longitude);
	nh.getParam("start_latitude", mapOrigin.latitude);
}