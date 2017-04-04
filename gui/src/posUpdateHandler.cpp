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
	WDtimeoutWarning.msgType = "wd_timeout";
	WDtimeoutWarning.obstacleID = obstID;
	pub.publish(WDtimeoutWarning);
}

void watchDog::run()
{
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
	qDebug() << "posUpdateHandler initialized" << QThread::currentThreadId();
}

posUpdateHandler::posUpdateHandler(const posUpdateHandler& other)
{
	sv = other.sv;
	obstUpdateSub = other.obstUpdateSub;
	gpsSub = other.gpsSub;
	qDebug() << "posUpdateHandler copied.";
}


void posUpdateHandler::run()
{
	WDthread = new QThread();
	WDthread->start();
	obstUpdateSub = nh.subscribe("obstUpdateTopic", 1000, &posUpdateHandler::obstUpdateParser, this);
	gpsSub = nh.subscribe("sensors/gps", 1000, &posUpdateHandler::gpsParser, this);
	qDebug() << "posUpdateHandler running from thread" << QThread::currentThreadId();
	ros::spin();
	qDebug() << "posUpdateHandler finished...";
}


void posUpdateHandler::obstUpdateParser(const environment::obstacleUpdate::ConstPtr& updateMsg)
{	
	string ID = updateMsg->obstacleID;

	if(updateMsg->msgType == "position_update")
	{
		double x = updateMsg->x;
		double y = updateMsg->y;
		double psi = updateMsg->psi;
		sv->setPosition(ID, x, y, psi);
		map<string, watchDog*>::iterator it = obstWDs.find(ID);
		if( it == obstWDs.end() ) //Could not find obstacle
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
	else if(updateMsg->msgType == "wd_timeout")
	{
		qDebug() << "Timout detected for obstacle" << updateMsg->obstacleID.c_str();

		obstWDs[ID]->quit();
		obstWDs[ID]->wait();
		obstWDs.erase(ID);
		sv->deleteObstacle(ID);
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

<<<<<<< HEAD
	sv->simTargetMoveTo(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->heading);
=======
	sv->simTargetMoveTo(gpsMsg->longitude, gpsMsg->latitude, gpsMsg->heading);
>>>>>>> 6c89fbd16b6318df98e7bcd588faf3cb012e65ac
	headingPlot->updateValues(gpsMsg->heading, gpsMsg->heading - 1);
	velocityPlot->updateValues(gpsMsg->speed, gpsMsg->speed - 0.5);
	//qDebug() << "Received new gps msg. Coord: " << gpsMsg->latitude << gpsMsg->longitude;
}

