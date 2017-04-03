
#include "sensorsim.h"
#include "simulator_messages/Gps.h"

//----------------------------------------------------------------------------------------
//----------------------------------------sensorSim----------------------------------------
//----------------------------------------------------------------------------------------


sensorSim::sensorSim(ros::NodeHandle n, QThread *parent) : QThread(parent)
{
	nh = n;
}

sensorSim::~sensorSim()
{
	delete AIStimer;
	delete DTtimer;
}

void sensorSim::run()
{
	ros::Subscriber gpsUpdateSub = nh.subscribe("sensors/gps", 1000, &sensorSim::gps_parser, this);

	AIStimer = new QTimer();
	DTtimer = new QTimer();
	QObject::connect( AIStimer, SIGNAL(timeout()), this, SLOT(publish_AIS()) );
	QObject::connect( DTtimer, SIGNAL(timeout()), this, SLOT(publish_detected_target_msg()) );
	AIStimer->start(1000);
	//DTtimer->start(1000);
	qDebug() << "Sensor simulator running...";
	QThread::exec();
}


void sensorSim::publish_AIS()
{
	qDebug() << "Publish new AIS message.";
}

void sensorSim::publish_detected_target_msg()
{
	qDebug() << "Publish new detected target message.";
}


void sensorSim::gps_parser(const simulator_messages::Gps::ConstPtr& USVgpsMsg)
{
	USVpos = gpsData(USVgpsMsg->longitude, USVgpsMsg->latitude, USVgpsMsg->heading, USVgpsMsg->headingRate, USVgpsMsg->speed, USVgpsMsg->altitude);
	qDebug() << "Received new gps msg. Coord: " << USVgpsMsg->latitude << USVgpsMsg->longitude;
}
