#include <thread>
#include <QApplication>
#include <QObject>

#include "ros/ros.h"

#include "obstacleControl.h"
#include "sensorsim.h"


using namespace std;

int main(int argc, char *argv[])
{

	QApplication a(argc, argv);

	ros::init(argc, argv, "obstacleControl");
	ros::NodeHandle nh;

	QThread *sensorThread = new QThread();
    sensorSim *sensorSimulator = new sensorSim(nh);
    sensorSimulator->moveToThread(sensorThread);
    QObject::connect(sensorThread, SIGNAL(finished()), sensorSimulator, SLOT(deleteLater()) );
    sensorThread->start();
    sensorSimulator->start();

	thread obstacleControl = thread(&obstacleHandler::run, obstacleHandler(nh));

	ROS_INFO("Environment initialized and running...");
	ros::Rate loop_rate(1);
	while(true)
	{
		loop_rate.sleep();
	}

	obstacleControl.join();
	
    return a.exec();
}


// #include "obstacleControl.h"
// #include "ros/ros.h"

// int main(int argc, char *argv[])
// {

// 	ros::init(argc, argv, "obstacleControl");
// 	ros::NodeHandle nh;
// 	sensor
// 	obstacleHandler myObstHandler(nh);
// 	ROS_INFO("Here...");
// 	myObstHandler.run();
// 	//obstacleControl(argc, argv);
// 	ros::Rate loop_rate(1);
// 	while(true)
// 	{
// 		loop_rate.sleep();
// 	}	
//     return 0;
// }
