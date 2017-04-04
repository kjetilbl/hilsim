<<<<<<< HEAD
#include "obstacleControl.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{


=======
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
>>>>>>> 6c89fbd16b6318df98e7bcd588faf3cb012e65ac

	ros::init(argc, argv, "obstacleControl");
	ros::NodeHandle nh;

<<<<<<< HEAD
	obstacleHandler myObstHandler(nh);
	myObstHandler.run();
	//obstacleControl(argc, argv);
	ROS_INFO("Here...");
=======
	QThread *sensorThread = new QThread();
    sensorSim *sensorSimulator = new sensorSim(nh);
    sensorSimulator->moveToThread(sensorThread);
    QObject::connect(sensorThread, SIGNAL(finished()), sensorSimulator, SLOT(deleteLater()) );
    sensorThread->start();
    sensorSimulator->start();

	thread obstacleControl = thread(&obstacleHandler::run, obstacleHandler(nh));

	ROS_INFO("Environment initialized and running...");
>>>>>>> 6c89fbd16b6318df98e7bcd588faf3cb012e65ac
	ros::Rate loop_rate(1);
	while(true)
	{
		loop_rate.sleep();
	}
<<<<<<< HEAD
	
    return 0;
=======

	obstacleControl.join();
	
    return a.exec();
>>>>>>> 6c89fbd16b6318df98e7bcd588faf3cb012e65ac
}