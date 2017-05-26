#include "posUpdateHandler.h"
#include <QTime>


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
	detectedTargetSub = nh.subscribe("sensors/target_detection", 1000, &posUpdateHandler::detectedTargetParser, this);

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
		double crossSection = updateMsg->size;
		
		gpsPointStamped pos(updateMsg->longitude, updateMsg->latitude, updateMsg->heading);

		if( sv->doesExist(ID) )
		{
			sv->setPosition(ID, pos);
		}else
		{
			string objectDescriptor = updateMsg->objectDescriptor;
			sv->addSimObject(ID, objectDescriptor, longitude, latitude, heading);
		}

		rviz->set_object(ID, gpsPoint3DOF{longitude, latitude, heading}, crossSection);
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



void posUpdateHandler::detectedTargetParser(const simulator_messages::detectedTarget::ConstPtr& dtMsg)
{	
	/*
	static QTime lastTime = QTime::currentTime();
	static int count = 0;
	QTime now = QTime::currentTime();
	if (lastTime.msecsTo(now) > 700){
		qDebug() << "GUI received" << count << "detected objects.";
		count = 0;
	}
	count++;
	lastTime = now;
	*/

	int targetID = dtMsg->targetID;
	string objectDescriptor = dtMsg->objectDescriptor;
	gpsPointStamped pos(dtMsg->longitude, dtMsg->latitude, dtMsg->COG);
	double SOG = dtMsg->SOG;
	double crossSection = dtMsg->crossSection;
	rviz->show_detected_target(targetID, objectDescriptor, pos, SOG, crossSection);
}

