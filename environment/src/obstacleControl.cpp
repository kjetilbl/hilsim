#include "obstacleControl.h"
#include <stdlib.h>
#include <math.h>

gpsPoint get_coordinate_from_string(string coordAsString){
	stringstream ss;
	ss.str(coordAsString);
	string longitudeAsString;
	string latitudeAsString;
	getline(ss,longitudeAsString, '/');
	getline(ss,latitudeAsString, '/');
	double longitude = atof(longitudeAsString.c_str());
	double latitude = atof(latitudeAsString.c_str());
	if(abs(longitude) > 180 || abs(latitude) > 90){
		qDebug() << "Invalid coordinate" << longitude << latitude;
		return gpsPoint(0,0);
	}
	return gpsPoint(longitude, latitude);
}

vector<string> split_string(string myString, char splitChar){
	stringstream ss;
    ss.str(myString);
    string tempString;
    vector<string> v;;
   	while( getline(ss, tempString, splitChar)){
   		v.push_back(tempString);
   	}
   	return v;
}


obstacleHandler::obstacleHandler(ros::NodeHandle *n, QThread *parent ) : QThread(parent) {
	this->nh = n;
	this->cmdSub = nh->subscribe("/simObject/command", 1000, &obstacleHandler::command_parser, this);
	get_origin_from_sim_params(*nh);
	
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
		spawn_fixed_obstacle(eta0, 20);
	}
}

void obstacleHandler::run()
{
	simObjectsThread = new QThread();
	simObjectsThread->start();

	//spawn_obstacles();

	// ros::AsyncSpinner spinner(1);
	// spinner.start();
	QThread::exec();
}

void obstacleHandler::spawn_obstacles(){

	// Spawn ships requested in .yaml file
	map<string, string> requestedShips;
	nh->getParam("ships", requestedShips);

	for(auto const& shipDescriptor: requestedShips){
	    vector<gpsPoint> waypoints;
	    double size = 1;
	    double speed = 0;

	    vector<string> shipParameters = split_string(shipDescriptor.second, ' ');
	    for (auto const& shipParam: shipParameters)
	    {
	    	vector<string> paramPair = split_string(shipParam, '=');
	    	if (paramPair.size() != 2)
	    	{
	    		qDebug() << "Error in ship parameters.";
	    	}
	    	string key = paramPair.front();
	    	string value = paramPair.back();
	   		if (key == "WP")
	   		{
	   			waypoints.push_back( get_coordinate_from_string(value) );
	   		}
	   		else if (key == "Size")
	   		{
	   			size = atof(value.c_str());
	   		}
	   		else if (key == "SpeedInKnots")
	   		{
	   			speed = atof(value.c_str());
	   		}
	   		else{
	   			qDebug() << "Unknown key" << key.c_str() << "in ship parameters.";
	   		}
	    }
	    if(waypoints.empty()){
	    	qDebug() << "Error in ship parameters. Ship must have at least one waypoint.";
	    }
	    else{
	    	gpsPoint3DOF eta0(waypoints.front().longitude, waypoints.front().latitude, 0);
	    	waypoints.erase(waypoints.begin());
		   	ship* newShip = new ship(nh, eta0, pow(size,2), speed);
		   	for (auto const& WP: waypoints)
		   	{
		   		newShip->add_waypoint(WP);
		   	}

		   	newShip->moveToThread(simObjectsThread);
			QObject::connect(simObjectsThread, SIGNAL(finished()), newShip, SLOT(deleteLater()) );
			newShip->start();
			simObjects.push_back( newShip );
	    }
	}


	// Spawn obstacles in random places. As many as requested in .yaml file
	gpsPoint3DOF eta0;
	int numberOfObstacles;
	nh->getParam("n_fixed_obstacles", numberOfObstacles);
	for(int i = 0; i < numberOfObstacles; i++){
		eta0.longitude = mapOrigin.longitude + (rand()%10000 - 5000)*longitude_degs_pr_meter(mapOrigin.latitude);
		eta0.latitude = mapOrigin.latitude + (rand()%10000 - 5000)*latitude_degs_pr_meter();
		uint16_t size = 500 + (rand() % 1000 - 400);
		spawn_fixed_obstacle(eta0, size);
	}
}


void obstacleHandler::spawn_fixed_obstacle(gpsPoint3DOF eta, double size){
	if ( simObjectsThread == NULL )
	{
		simObjectsThread = new QThread();
		simObjectsThread->start();
	}
	fixedObstacle* newObstacle = new fixedObstacle(this->nh, eta, size);
	newObstacle->moveToThread( this->simObjectsThread );
	QObject::connect(this->simObjectsThread, SIGNAL(finished()), newObstacle, SLOT(deleteLater()) );
	newObstacle->start();
	this->simObjects.push_back( newObstacle );
}


void obstacleHandler::get_origin_from_sim_params(ros::NodeHandle nh){
	nh.getParam("start_longitude", mapOrigin.longitude);
	nh.getParam("start_latitude", mapOrigin.latitude);
}