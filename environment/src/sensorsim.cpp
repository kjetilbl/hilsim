
#include "sensorsim.h"
#include "simulator_messages/Gps.h"
#include <QTime>
#include "stdint.h"
#include <math.h>
#include <random>

#define ASCII_CHAR_OFFSET 	48
#define SIX_BIT_MASK 		0b00111111
#define FIVE_BIT_MASK 		0b00011111
#define FOUR_BIT_MASK 		0b00001111
#define THREE_BIT_MASK 		0b00000111
#define TWO_BIT_MASK 		0b00000011
#define ONE_BIT_MASK 		0b00000001

uint32_t detectedObject::targetIterator = 0;

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
	int count = 0;
	// Print obstacle position info and find outdated obstacles
	for( auto & obst : unidentifiedObjects )
	{
		string obstID = obst.first;
		if( obst.second.msecs_since_last_update() > 2000)
		{
			outdatedObstacles.push_back(obstID);
		}
		else
		{
			double distanceToObject = distance_m(USVnavData.get_position(), obst.second.get_true_position());
			obst.second.make_parameter_estimates(distanceToObject);
			simulator_messages::detectedTarget dt = obst.second.makeDTmsg();
			detectedTargetPub.publish(dt);
			count++;
			//qDebug() << "DT: " << dt.targetID.c_str() << dt.longitude << dt.latitude;
			/* 
			qDebug() << obst.first.c_str() << ":";
			obst.second.print();
			qDebug() << "------------------------";
			*/
		}
	}
	qDebug() << "sensorSim sent" << count << "detected objects";
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

	gpsPoint obstPos(obstUpdateMsg->longitude, obstUpdateMsg->latitude);
	double crossSection = obstUpdateMsg->size;
	double COG = obstUpdateMsg->heading;

	if( obstUpdateMsg->msgDescriptor == "position_update" )
	{
		if(obstUpdateMsg->objectDescriptor == "fixed_obstacle"){
			string objectID = obstUpdateMsg->objectID;
			if( !is_within_visibility( objectID, obstPos, crossSection) ){
				return;
			}
			double SOG = 0;
			map<string, detectedObject>::iterator it = unidentifiedObjects.find(objectID);
			if( it != unidentifiedObjects.end() ) // Object already detected
			{
				unidentifiedObjects[objectID].update_true_states(obstPos, COG, SOG, crossSection);
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

bool sensorSim::is_within_visibility(string ID, gpsPoint newPos, double crossSection){
	//Check range
	gpsPointStamped usvPos = USVnavData.get_position();
	double distToNewPos = distance_m(usvPos, newPos);
	if (distToNewPos > radarRange)
	{
		return false;
	}
	if (sqrt(crossSection)/distToNewPos < 0.01)
	{
		return false;
	}

	// Check if behind other objects
	for( auto & mapPair : unidentifiedObjects ){
		gpsPoint objectPos = mapPair.second.get_true_position();
		double distToObject = distance_m(usvPos, objectPos);
		double objectRadius = sqrt(mapPair.second.get_true_CS())/2;
		if( distToNewPos < distToObject ) // newPos is closer
		{
			continue;
		}
		if( (distToNewPos > distToObject - 1) && (distToNewPos < distToObject + 1)){
			continue;
		}
		double newPosBearing = compass_bearing(usvPos, newPos);
		double maxBearing = compass_bearing(usvPos, objectPos) + atan2(objectRadius, distToObject)*180/M_PI;
		double minBearing = compass_bearing(usvPos, objectPos) - atan2(objectRadius, distToObject)*180/M_PI;
		if ( is_within_bearing_range(newPosBearing, minBearing, maxBearing) )
		{
			return false;
		}
	}

	//
	return true;
}

detectedObject::detectedObject(){}

detectedObject::detectedObject(gpsPoint truePosition, 
								double trueCOG, 
								double trueSOG, 
								double trueCrossSection){
	targetNumber = targetIterator++;
	int n = 5;

	X = Eigen::VectorXd(n);
	update_true_states(truePosition, trueCOG, trueSOG, trueCrossSection);

	Xm = Eigen::VectorXd(n);
	b = Eigen::VectorXd::Zero(n);
	biasSigmas = Eigen::VectorXd(n);
	biasSigmas << 	0.4*longitude_degs_pr_meter(X(1)), 
					0.4*latitude_degs_pr_meter(), 
					2, 
					0.5, 
					1;
	measureSigmas = Eigen::VectorXd(n);
	measureSigmas << 0, 0, 0, 0, 0;
	Eigen::VectorXd biasEigenValues(n);
	biasEigenValues << -2, -2, -0.0005, -0.5, -0.01;
	T = biasEigenValues.asDiagonal();
	Td = Eigen::MatrixXd::Zero(n, n); // discretize at each time step (varying sample time)
	errorPrDistanceGain = Eigen::MatrixXd(n,n);
	errorPrDistanceGain << 	2, 	0,	0,	0,	0,
							0,	2,	0,	0,	0,
							0,	0, 	2,	0,	0,
							0,	0,	0,	1,	0,
							0,	0,	0,	0,	3;
	errorPrDistanceGain /= 1000;	
	make_parameter_estimates(1);
}

void detectedObject::make_parameter_estimates( double distanceFromUSV_m ){
	int n = Xm.rows();
	static QTime lastTime = QTime::currentTime();
	static std::default_random_engine randomGenerator;
	static std::normal_distribution<double> gaussianWhiteNoise(0,1);
	
	QTime now = QTime::currentTime();
	double dt = (double)(lastTime.msecsTo( now ))/1000;

	Eigen::VectorXd e(n);
	Eigen::VectorXd w(n);
	Eigen::VectorXd v(n);
	for (int i = 0; i < n; i++)
	{
		Td(i,i) = exp(T(i,i)*dt);
		w(i) = gaussianWhiteNoise(randomGenerator)*biasSigmas(i);
		v(i) = gaussianWhiteNoise(randomGenerator)*measureSigmas(i);
	}

	// Make artificial deviations
	b = Td*b + w;
	e = b + v;
	e = errorPrDistanceGain * distanceFromUSV_m * e;
	e(3) = max(-X(3), (double)e(3));
	e(4) = max(-X(4) + 1, (double)e(4));

	// Update artificial measurements:
	Xm = X + e;

	//cout << endl << "Target " << targetNumber << ". Distance from USV: " << distanceFromUSV_m << ". e = " << endl << e << endl;

	lastTime = now;
}

void detectedObject::update_true_states(gpsPoint pos, double COG, double SOG, double crossSection){
	set_true_position(pos);
	set_true_COG(COG);
	set_true_SOG(SOG);
	set_true_cross_section(crossSection);
	lastUpdate = QTime::currentTime();
}

simulator_messages::detectedTarget detectedObject::makeDTmsg(){
	simulator_messages::detectedTarget dt;
	dt.targetID = "detected_target_" + to_string( get_target_number() );
	dt.longitude = get_estimated_position().longitude;
	dt.latitude = get_estimated_position().latitude;
	dt.COG = get_estimated_COG();
	dt.size = get_estimated_CS();
	return dt;
}

void detectedObject::set_true_position(gpsPoint truePos){
	X(0) = truePos.longitude;
	X(1) = truePos.latitude;
}

void detectedObject::set_true_COG(double trueCOG){
	X(2) = trueCOG;
}

void detectedObject::set_true_SOG(double trueSOG){
	X(3) = trueSOG;
}

void detectedObject::set_true_cross_section(double trueCS){
	X(4) = trueCS;
}
