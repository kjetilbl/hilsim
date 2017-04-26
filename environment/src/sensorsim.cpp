
#include "sensorsim.h"
#include "simulator_messages/Gps.h"
#include <QTime>
#include "stdint.h"
#include <math.h>

#define ASCII_CHAR_OFFSET 	48
#define SIX_BIT_MASK 		0b00111111
#define FIVE_BIT_MASK 		0b00011111
#define FOUR_BIT_MASK 		0b00001111
#define THREE_BIT_MASK 		0b00000111
#define TWO_BIT_MASK 		0b00000011
#define ONE_BIT_MASK 		0b00000001

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
	qDebug() << "sensorSim destructed...";
}

void sensorSim::run()
{
	USVnavData = navData(316001245, 123.8777500, 49.2002817, 19.6, 235.0);
	ros::Subscriber USVupdateSub = nh.subscribe("sensors/gps", 1000, &sensorSim::USV_gps_parser, this);
	ros::Subscriber obstUpdateSub = nh.subscribe("obstUpdateTopic", 1000, &sensorSim::obstacle_update_parser, this);

	AIStimer = new QTimer();
	DTtimer = new QTimer();
	QObject::connect( AIStimer, SIGNAL(timeout()), this, SLOT(print_USV_AIS_msg()) );
	QObject::connect( DTtimer, SIGNAL(timeout()), this, SLOT(print_detected_targets()) );
	AIStimer->start(2000);
	DTtimer->start(1000);
	qDebug() << "Sensor simulator running...";
	QThread::exec();
	qDebug() << "Sensor simulator finished executing...";
}


void sensorSim::print_USV_AIS_msg()
{
	std::lock_guard<std::mutex> lock(m);
	USVnavData.set_time(QTime::currentTime());
	qDebug() << "\n---------------------Publish USV AIS message---------------------";
	USVnavData.print_data();
	qDebug() << "-----------------------------------------------------------------\n";
}


void sensorSim::print_detected_targets()
{
	std::lock_guard<std::mutex> lock(m);
	if ( !obstPositions.empty() )
		qDebug() << "Printing all detected obstacles:";
	
	vector<string> outdatedObstacles;

	// Print obstacle position info and find outdated obstacles
	for( auto const& obst : obstPositions )
	{
		QTime now = QTime::currentTime();
		if( obst.second.time.msecsTo(now) > 1000)
		{
			string obstID = obst.first;
			outdatedObstacles.push_back(obstID);
		}
		else
		{
			qDebug() << obst.first.c_str() << ":";
			obst.second.print();
			qDebug() << "------------------------";
		}
	}

	// Delete old obstacles
	for ( auto const& ID : outdatedObstacles ) 
	{
    	obstPositions.erase(ID);
		qDebug() << "Erased " << ID.c_str() << "from map";
		qDebug() << "-------------------------";
	}
}


void sensorSim::USV_gps_parser(const simulator_messages::Gps::ConstPtr& USVgpsMsg)
{
	std::lock_guard<std::mutex> lock(m);
	USVnavData.set_position_accuracy(HIGH);
	USVnavData.set_nav_status(UNDERWAY_USING_ENGINE);
	USVnavData.set_position(USVgpsMsg->longitude, USVgpsMsg->latitude);
	USVnavData.set_track(USVgpsMsg->heading);
	USVnavData.set_COG(USVgpsMsg->heading);
	USVnavData.set_ROT(USVgpsMsg->headingRate);
	USVnavData.set_SOG(USVgpsMsg->speed);
	USVnavData.set_time(QTime::currentTime());
}

void sensorSim::obstacle_update_parser(const environment::obstacleUpdate::ConstPtr& obstUpdateMsg)
{
	std::lock_guard<std::mutex> lock(m);
	if( obstUpdateMsg->msgDescriptor == "position_update" )
	{
		string ID = obstUpdateMsg->objectID;
		obstPositions[ID] = gpsData(obstUpdateMsg->longitude, obstUpdateMsg->latitude, obstUpdateMsg->heading);		
	}

}
