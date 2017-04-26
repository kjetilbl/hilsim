#include <thread>
#include <QApplication>
#include <QObject>
#include <QDebug>

#include "ros/ros.h"

#include "obstacleControl.h"
#include "sensorsim.h"
#include <signal.h>

using namespace std;

static void quit_application(int sig)
{
    qDebug() << "\n-----------------------------------------------";
	qDebug() << "Quit Environment Application";
	qApp->quit();
}

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
    QThread *obstacleThread = new QThread();
    obstacleHandler *oh = new obstacleHandler(nh);
    oh->moveToThread(obstacleThread);
    QObject::connect(obstacleThread, SIGNAL(finished()), oh, SLOT(deleteLater()) );
    obstacleThread->start();
    oh->start();
	//thread obstacleControl = thread(&obstacleHandler::run, obstacleHandler(nh));

	qDebug() << "Environment Simulator initialized...";

    signal(SIGINT, quit_application);
	
    int exitCode = a.exec();

    oh->quit();
    oh->wait();
    sensorSimulator->quit();
    sensorSimulator->wait();

    obstacleThread->quit();
    obstacleThread->wait();
    sensorThread->quit();
    sensorThread->wait();

    qDebug() << "Environment Simulator finished with exit code" << exitCode;
    qDebug() << "-----------------------------------------------";
    return exitCode;
}
