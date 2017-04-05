#include <thread>
#include <QApplication>
#include <QObject>
#include <QDebug>

#include "ros/ros.h"

#include "obstacleControl.h"
#include "sensorsim.h"


using namespace std;

int main(int argc, char *argv[])
{

	QApplication a(argc, argv);

	ros::init(argc, argv, "obstacleControl");
	ros::NodeHandle nh;

	// Start simulation of sensors:
	QThread *sensorThread = new QThread();
    sensorSim *sensorSimulator = new sensorSim(nh);
    sensorSimulator->moveToThread(sensorThread);
    QObject::connect(sensorThread, SIGNAL(finished()), sensorSimulator, SLOT(deleteLater()) );
    sensorThread->start();
    sensorSimulator->start();

    // Start simulation of obstacles:
	thread obstacleControl = thread(&obstacleHandler::run, obstacleHandler(nh));

	qDebug() << "Ocean environment initialized...";

	obstacleControl.join();
	
    return a.exec();
}
