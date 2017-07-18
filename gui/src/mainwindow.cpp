#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <unistd.h>

#include "/opt/ros/kinetic/include/ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "simulator_messages/obstacleUpdate.h"


MainWindow::MainWindow(ros::NodeHandle *nh, QWidget *parent) :
	QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	setWindowTitle("USV HIL Simulator");

	headingPlot = new realtimePlot(ui->headingPlot);
	headingPlot->title("Heading");
	headingPlot->ylabel("Heading");
	headingPlot->updateValues(3);

	velocityPlot = new realtimePlot(ui->velocityPlot);
	velocityPlot->title("Velocity");
	velocityPlot->ylabel("Velocity");
	velocityPlot->updateValues(1);

	sv = new satelliteView(ui->satelliteView);

	obstInterface = new obstacleInterface(nh, ui->obstInterfaceWindow, sv);

	puhThread = new QThread(this);
    puh = new posUpdateHandler(nh, sv, headingPlot, velocityPlot);
    puh->moveToThread(puhThread);
    QObject::connect(puhThread, SIGNAL(finished()), puh, SLOT(deleteLater()));
    puhThread->start();
    puh->start();
}


MainWindow::~MainWindow()
{
    delete ui;
    delete headingPlot;
    delete velocityPlot;
    delete sv;
    delete obstInterface;
	puh->quit();
	puh->wait();
	puhThread->quit();
	puhThread->wait();
	delete puhThread ;
}