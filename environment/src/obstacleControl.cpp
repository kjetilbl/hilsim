#include "obstacleControl.h"


obstacleHandler::obstacleHandler(ros::NodeHandle nh)
{
	n = nh;
	cmdSub = nh.subscribe("obstacleCommandTopic", 1000, &obstacleHandler::command_parser, this);

	simObjectsThread = new QThread();
}


void obstacleHandler::command_parser(const environment::obstacleCmd::ConstPtr& cmd)
{
	static int obstIterator = 0;
	if(cmd->cmdSpecifier == "spawn")
	{
		string ID = "fixedObstacle_" + to_string(obstIterator++);
		double x = cmd->x;
		double y = cmd->y;
		double psi = cmd->psi;
		ROS_INFO("%s %s [%f %f %f]", cmd->cmdSpecifier.c_str(), cmd->receiverID.c_str(), x,y,psi);

		fixedObstacle* newObstacle = new fixedObstacle(n, ID, x, y, psi);
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
	x = other.x;
	y = other.y;
	psi = other.psi;
}

simObject::simObject(ros::NodeHandle nh, string obstID, double X, double Y, double Psi, QThread *parent) : QThread(parent)
{
	n = nh;
	cmdSub = n.subscribe("obstacleCommandTopic", 1000, &simObject::command_parser, this);
	posUpdatePub = n.advertise<environment::obstacleUpdate>("obstUpdateTopic", 1000);

	ID = obstID;
	x = X;
	y = Y;
	psi = Psi;

}

void simObject::run()
{
	ros::Rate loop_rate(24);
	while(true){
		move();
		publish_position_report();

		loop_rate.sleep();

		lock_guard<mutex> lock(m);
		if(stop == true)
		{
			return;
		}
	}
	ROS_INFO("Will terminate simObject %s", ID.c_str());
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

	environment::obstacleUpdate obstUpdate;
	obstUpdate.msgType = "position_update";
	obstUpdate.obstacleID = ID;
	obstUpdate.x = x;
	obstUpdate.y = y;
	obstUpdate.psi = psi;
	return obstUpdate;
}


fixedObstacle::fixedObstacle( ros::NodeHandle nh, string obstID, double X, double Y, double Psi, QThread *parent ) 
							: simObject( nh, obstID, X, Y, Psi, parent )
{

}


void fixedObstacle::move()
{
	// dont move...
}

ship::ship( ros::NodeHandle nh, string obstID, double X, double Y, double Psi, QThread *parent ) 
				: simObject( nh, obstID, X, Y, Psi, parent )
{

}


void ship::move()
{
	x += 0.00001;
	y += 0.00001;
}