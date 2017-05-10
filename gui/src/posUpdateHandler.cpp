#include "posUpdateHandler.h"


posUpdateHandler::posUpdateHandler(ros::NodeHandle n, satelliteView *Sv, realtimePlot *hdngPlot, realtimePlot *velPlot)
{
	sv = Sv;
	nh = n;
	headingPlot = hdngPlot;
	velocityPlot = velPlot;
	rviz = new rvizInterface(n);
}

posUpdateHandler::posUpdateHandler(const posUpdateHandler& other)
{
	sv = other.sv;
	obstUpdateSub = other.obstUpdateSub;
	gpsSub = other.gpsSub;
}

posUpdateHandler::~posUpdateHandler(){
	delete rviz;
}


void posUpdateHandler::run()
{
	obstUpdateSub = nh.subscribe("/simObject/position", 1000, &posUpdateHandler::obstUpdateParser, this);
	gpsSub = nh.subscribe("sensors/gps", 1000, &posUpdateHandler::gpsParser, this);
	
	ros::AsyncSpinner spinner(1);
	spinner.start();
	QThread::exec();}


void posUpdateHandler::obstUpdateParser(const environment::obstacleUpdate::ConstPtr& updateMsg)
{	
	string ID = updateMsg->objectID;

	if(updateMsg->msgDescriptor == "position_update")
	{
		double longitude = updateMsg->longitude;
		double latitude = updateMsg->latitude;
		double heading = updateMsg->heading;
		
		gpsPointStamped pos(updateMsg->longitude, updateMsg->latitude, updateMsg->heading);

		if( sv->doesExist(ID) )
		{
			sv->setPosition(ID, pos);
		}else
		{
			string objectDescriptor = updateMsg->objectDescriptor;
			sv->addSimObject(ID, objectDescriptor, longitude, latitude, heading);
		}

		rviz->set_object(ID, gpsPoint3DOF{longitude, latitude, heading});
	}
}


void posUpdateHandler::gpsParser(const simulator_messages::Gps::ConstPtr& gpsMsg)
{
	static bool firstContact = true;
	if(firstContact)
	{
		sv->simTargetClearTrajectory();
		headingPlot->clear();
		velocityPlot->clear();
		firstContact = false;
	}
	gpsPointStamped pos(gpsMsg->longitude, gpsMsg->latitude, gpsMsg->heading);
	sv->simTargetMoveTo(pos);
	headingPlot->updateValues(gpsMsg->heading, gpsMsg->heading - 1);
	velocityPlot->updateValues(gpsMsg->speed, gpsMsg->speed - 0.5);
}

