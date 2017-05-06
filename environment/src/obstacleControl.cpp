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
		ROS_INFO("%s %s [%f %f %f]", cmd->cmdSpecifier.c_str(), cmd->receiverID.c_str(), cmd->x, cmd->y, cmd->psi);

		gpsPoint3DOF eta0{cmd->x, cmd->y, cmd->psi};
		
		ship* newObstacle = new ship(n, obstIterator++, eta0);
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
		gpsPoint3DOF eta0{mapOrigin.longitude, mapOrigin.latitude, 45*i};
		ship* newShip = new ship(n, i, eta0);
		gpsPoint firstWaypoint = eta0;
		firstWaypoint.latitude += 50*latitude_degs_pr_meter(); //m
		newShip->add_waypoint(firstWaypoint);
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

