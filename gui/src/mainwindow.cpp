#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <QThread>
#include <thread>
#include <unistd.h>

#include "/opt/ros/kinetic/include/ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "environment/obstacleUpdate.h"


MainWindow::MainWindow(int argc, char *argv[], QWidget *parent) :
	QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	setWindowTitle("USV HIL Simulator");

	headingPlot = new realtimePlot(ui->headingPlot);
	headingPlot->title("Heading");
	headingPlot->ylabel("Heading");
	headingPlot->updateValues(3,4);

	velocityPlot = new realtimePlot(ui->velocityPlot);
	velocityPlot->title("Velocity");
	velocityPlot->ylabel("Velocity");
	velocityPlot->updateValues(1,2);

	sv = new satelliteView(ui->satelliteView);

	ros::init(argc, argv, "gui");
	ros::NodeHandle nh;

	obstInterface = new obstacleInterface(nh, ui->obstInterfaceWindow, sv);

	puhThread = new QThread(this);
    this->puh = new posUpdateHandler(nh, sv, headingPlot, velocityPlot);
    this->puh->moveToThread(puhThread);
    connect(puhThread, SIGNAL(finished()), this->puh, SLOT(deleteLater()) );
    puhThread->start();
    puh->start();
}


//TODO: fullføre destruktor: kanskje årsak til qthread still running...
MainWindow::~MainWindow()
{
    delete ui;
}