#include <thread>
#include <QApplication>
#include <QObject>
#include <QDebug>

#include "ros/ros.h"

#include "obstacleManager.h"
#include "targetDetection.h"
#include <signal.h>

using namespace std;

static void quit_application(int sig)
{
	qApp->quit();
}

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	ros::init(argc, argv, "obstacleManager");
	ros::NodeHandle nh;

	// Start simulation of sensors:
	QThread *sensorThread = new QThread();
    targetDetectionModule *targetDetectionModuleulator = new targetDetectionModule(&nh);
    targetDetectionModuleulator->moveToThread(sensorThread);
    QObject::connect(sensorThread, SIGNAL(finished()), targetDetectionModuleulator, SLOT(deleteLater()) );
    sensorThread->start();
    targetDetectionModuleulator->start();

    // Start simulation of obstacles:
    QThread *obstacleThread = new QThread();
    obstacleManager *oh = new obstacleManager(&nh);
    oh->moveToThread(obstacleThread);
    QObject::connect(obstacleThread, SIGNAL(finished()), oh, SLOT(deleteLater()) );
    obstacleThread->start();
    oh->start();

	qDebug() << "Environment Simulator initialized...";

    signal(SIGINT, quit_application);
	
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int exitCode = a.exec();
    //------------------------------------------------

    oh->quit();
    oh->wait();
    targetDetectionModuleulator->quit();
    targetDetectionModuleulator->wait();

    obstacleThread->quit();
    obstacleThread->wait();

    sensorThread->quit();
    sensorThread->wait();

    qDebug() << "Environment Simulator finished with exit code" << exitCode;
    return exitCode;
}
