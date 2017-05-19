
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

uint32_t detectedObject::targetIterator = 0;

simulator_messages::detectedTarget makeDTmsg(detectedObject obj){
	simulator_messages::detectedTarget dt;
	dt.targetID = "detected_target_" + to_string(obj.get_target_number());
	dt.longitude = obj.get_estimated_position().longitude;
	dt.latitude = obj.get_estimated_position().latitude;
	dt.COG = obj.get_estimated_COG();
	dt.size = obj.get_estimated_CS();
	return dt;
}

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
	USVnavData = navData(316001245, 123.8777500, 49.2002817, 19.6, 235.0);
	ros::Subscriber USVupdateSub = nh.subscribe("sensors/gps", 1000, &sensorSim::USV_gps_parser, this);
	ros::Subscriber obstUpdateSub = nh.subscribe("/simObject/position", 1000, &sensorSim::obstacle_update_parser, this);
	ros::Subscriber AISsub = nh.subscribe("sensors/ais", 1000, &sensorSim::AIS_parser, this);
	detectedTargetPub = nh.advertise<simulator_messages::detectedTarget>("sensors/target_detection", 1000);

	AIStimer = new QTimer();
	DTtimer = new QTimer();
	QObject::connect( AIStimer, SIGNAL(timeout()), this, SLOT(print_USV_AIS_msg()) );
	QObject::connect( DTtimer, SIGNAL(timeout()), this, SLOT(print_detected_targets()) );
	AIStimer->start(2000);
	DTtimer->start(500);

	ros::AsyncSpinner spinner(1);
	spinner.start();
	QThread::exec();
}


void sensorSim::print_USV_AIS_msg()
{
	std::lock_guard<std::mutex> lock(m);
	USVnavData.set_time(QTime::currentTime());
	/*
	qDebug() << "\n---------------------Publish USV AIS message---------------------";
	USVnavData.print_data();
	qDebug() << "-----------------------------------------------------------------\n";
	*/
}


void sensorSim::print_detected_targets()
{
	std::lock_guard<std::mutex> lock(m);
	if ( !unidentifiedObjects.empty() ){
		// qDebug() << "Printing all detected obstacles:";
	}
	
	vector<string> outdatedObstacles;

	// Print obstacle position info and find outdated obstacles
	for( auto & obst : unidentifiedObjects )
	{
		QTime now = QTime::currentTime();
		string obstID = obst.first;
		if( obst.second.truePos.timeStamp.msecsTo(now) > 2000)
		{
			outdatedObstacles.push_back(obstID);
		}
		else
		{
			obst.second.make_parameter_estimates(distance_m(USVnavData.get_position(), obst.second.truePos));
			simulator_messages::detectedTarget dt = makeDTmsg(obst.second);
			detectedTargetPub.publish(dt);
			//qDebug() << "DT: " << dt.targetID.c_str() << dt.longitude << dt.latitude;
			/* 
			qDebug() << obst.first.c_str() << ":";
			obst.second.print();
			qDebug() << "------------------------";
			*/
		}
	}

	// Delete old obstacles
	for ( auto const& ID : outdatedObstacles ) 
	{
    	unidentifiedObjects.erase(ID);
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
	gpsPointStamped obstPos(obstUpdateMsg->longitude, obstUpdateMsg->latitude, obstUpdateMsg->heading);
	double crossSection = obstUpdateMsg->size;
	double COG = obstUpdateMsg->heading;
	double SOG = 0;

	if( obstUpdateMsg->msgDescriptor == "position_update" )
	{
		if(obstUpdateMsg->objectDescriptor == "fixed_obstacle"){

			if (!is_within_visibility(obstPos, obstUpdateMsg->size))
			{
				return;
			}

			string objectID = obstUpdateMsg->objectID;
			map<string, detectedObject>::iterator it = unidentifiedObjects.find(objectID);
			if( it != unidentifiedObjects.end() )
			{
				// Object already detected
				unidentifiedObjects[objectID].truePos = obstPos;
				unidentifiedObjects[objectID].trueCS = crossSection;
				unidentifiedObjects[objectID].trueCOG = COG;
				unidentifiedObjects[objectID].trueSOG = SOG;
			}else
			{
				unidentifiedObjects[objectID] = detectedObject(obstPos, crossSection, COG, SOG);
			}
		}
	}
}

void sensorSim::AIS_parser(const simulator_messages::AIS::ConstPtr& AISmsg)
{
	std::lock_guard<std::mutex> lock(m);
	navData nd(AISmsg->MMSI, AISmsg->longitude, AISmsg->latitude, AISmsg->SOG, AISmsg->track);
	nd.set_nav_status((navStatus)AISmsg->status);
	nd.set_ROT(AISmsg->ROT);
	nd.set_position_accuracy((posAccuracy)AISmsg->positionAccuracy);
	nd.set_COG(AISmsg->COG);
	nd.set_time(QTime(AISmsg->hour, AISmsg->minute, AISmsg->second));
	detectedAISusers[AISmsg->MMSI] = nd;

	//TODO: sende disse over til gui som plotter i rviz
}

bool sensorSim::is_within_visibility(gpsPoint obstaclePosition, double crossSection){
	//Check range
	gpsPointStamped usvPos = USVnavData.get_position();
	double distanceToObject = distance_m(usvPos, obstaclePosition);
	if (distanceToObject > radarRange)
	{
		return false;
	}
	if (crossSection/distanceToObject < 0.01)
	{
		return false;
	}

	// Check if behind other objects

	//
	return true;
}

detectedObject::detectedObject(gpsPointStamped truePosition, 
								double trueCrossSection, 
								double trueCourseOverGround, 
								double trueSpeedOverGround){
	truePos = truePosition;
	trueCS = trueCrossSection;
	trueCOG = trueCourseOverGround;
	trueSOG = trueSpeedOverGround;
	targetNumber = targetIterator++;
	make_parameter_estimates(1);
}

void detectedObject::make_parameter_estimates(double distanceFromUSV){
	estimatedPos = truePos;
	estimatedCS = trueCS;
	estimatedCOG = trueCOG;
	estimatedSOG = trueSOG;
}