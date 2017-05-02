#include "posUpdateHandler.h"


//----------------------------------------------------------------------------------------
//----------------------------------------watchDog----------------------------------------
//----------------------------------------------------------------------------------------


watchDog::watchDog(ros::NodeHandle n, string obstacleID, int msec, QThread *parent) : QThread(parent)
{
	nh = n;
	obstID = obstacleID;
	timeoutInterval = msec;
}

watchDog::~watchDog(){
	delete timer;
}

void watchDog::kick()
{
	timer->start(timeoutInterval);
}

void watchDog::timoutHandler()
{
	environment::obstacleUpdate WDtimeoutWarning;
	WDtimeoutWarning.msgDescriptor = "wd_timeout";
	WDtimeoutWarning.objectID = obstID;
	pub.publish(WDtimeoutWarning);
}

void watchDog::run()
{
	// TODO: bruke egen topic og msg til dette...
	pub = nh.advertise<environment::obstacleUpdate>("obstUpdateTopic", 1000);

	timer = new QTimer();
	connect( timer, SIGNAL(timeout()), this, SLOT(timoutHandler()) );
	timer->start(timeoutInterval);
	QThread::exec();
}

//----------------------------------------------------------------------------------------
//----------------------------------posUpdateHandler---------------------------------
//----------------------------------------------------------------------------------------



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
	for( auto const& wd : obstWDs ){
		wd.second->quit();
		wd.second->wait();
		delete wd.second;
	}

	if( WDthread != NULL ){
		WDthread->quit();
		WDthread->wait();
	}
}


void posUpdateHandler::run()
{
	WDthread = new QThread();
	WDthread->start();
	obstUpdateSub = nh.subscribe("obstUpdateTopic", 1000, &posUpdateHandler::obstUpdateParser, this);
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
		
		if( sv->doesExist(ID) )
		{
			sv->setPosition(ID, longitude, latitude, heading);
		}else
		{
			string objectDescriptor = updateMsg->objectDescriptor;
			sv->addSimObject(ID, objectDescriptor, longitude, latitude, heading);
		}

		rviz->set_object(ID, gpsPoint3DOF{longitude, latitude, heading});
		
		map<string, watchDog*>::iterator it = obstWDs.find(ID);
		if( it == obstWDs.end() ) // Could not find watchDog for this simObject
		{
			obstWDs[ID] = new watchDog(nh, ID);
			obstWDs[ID]->moveToThread(WDthread);
		    connect(WDthread, SIGNAL(finished()), obstWDs[ID], SLOT(deleteLater()) );
			obstWDs[ID]->start();
		}
		else
		{
			obstWDs[ID]->kick();
		}
	}
	else if(updateMsg->msgDescriptor == "wd_timeout")
	{
		obstWDs[ID]->quit();
		obstWDs[ID]->wait();
		obstWDs.erase(ID);
		sv->removeSimObject(ID);
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

	sv->simTargetMoveTo(gpsMsg->longitude, gpsMsg->latitude, gpsMsg->heading);
	headingPlot->updateValues(gpsMsg->heading, gpsMsg->heading - 1);
	velocityPlot->updateValues(gpsMsg->speed, gpsMsg->speed - 0.5);
}

