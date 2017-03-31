#include "satelliteview.h"
//#include <QtMath>
#include <QDebug>
#include <QPixmap>
#include <QFile>
#include <iostream>       // std::cout
#include <unistd.h>


satelliteView::satelliteView(QCustomPlot *satelliteViewWidget)
{
	// testObstacle = new obstacle("testObstacle", 1, 1);
	// testObstacle->makePlottable(satelliteViewWidget);
	// obstacles["testObstacle2"] = new obstacle("SomeID", -1, 1);
	// obstacles["testObstacle2"]->makePlottable(satelliteViewWidget);
	svWidget = satelliteViewWidget;

	svWidget->setBackground(Qt::lightGray);
	svWidget->axisRect()->setBackground(Qt::white);

	svWidget->axisRect()->setupFullAxesBox();
	svWidget->xAxis->setRange(-2, 2);
	svWidget->yAxis->setRange(-2, 2);

	// make left and bottom axes transfer their ranges to right and top axes:
	connect(svWidget->xAxis, SIGNAL(rangeChanged(QCPRange)), svWidget->xAxis2, SLOT(setRange(QCPRange)));
	connect(svWidget->yAxis, SIGNAL(rangeChanged(QCPRange)), svWidget->yAxis2, SLOT(setRange(QCPRange)));

	connect(&plotTimer, SIGNAL(timeout()), this, SLOT(updatePlot()));
	plotTimer.start(30); // Interval 0 means to refresh as fast as possible

	connect(svWidget, SIGNAL(mousePress(QMouseEvent*)), SLOT(markPosition( QMouseEvent *)));

	simTarget = new USV("USV_Odin", svWidget, 0, 0);
	simTarget->makePlottable();
	
}

void satelliteView::updatePlot()
{
	std::lock_guard<std::mutex> lock(mpMutex);
	static double y = 0;
	static double x = 0;
	static double psi = 0;
	y = y + 0.005;
	x = x + 0.005;
	psi = psi + 1;
	position newPos = {x,y,psi};
	//Odin->setNewPosition(newPos);
	simTarget->updateTrajectory();
	for( auto const& obst : obstacles )
	{
		if(!obst.second->isPlottable())
		{
			obst.second->makePlottable();	
		}
		obst.second->updateTrajectory();
	}

	position plotCenter = simTarget->getPosition();

	double zoom = 0.001;
	svWidget->xAxis->setRange(plotCenter.x - zoom, plotCenter.x + zoom);
	svWidget->yAxis->setRange(plotCenter.y - zoom, plotCenter.y + zoom);
	svWidget->replot();
}

void satelliteView::addObstacle( string obstacleID, double x, double y, double psi )
{
	std::lock_guard<std::mutex> lock(mpMutex);
	obstacles[obstacleID] = new obstacle(obstacleID.c_str(), svWidget, x, y, psi);
}

void satelliteView::setPosition( string obstacleID, double x, double y, double psi )
{
	position newPos = {x, y, psi};
	map<string, obstacle*>::iterator it = obstacles.find(obstacleID);
	if( it != obstacles.end() )
	{
		std::lock_guard<std::mutex> lock(mpMutex);
		obstacles[obstacleID]->setNewPosition(newPos);
	}else
	{
		addObstacle(obstacleID, x, y, psi);
	}
}

void satelliteView::deleteObstacle(string obstacleID)
{
	std::lock_guard<std::mutex> lock(mpMutex);
	map<string, obstacle*>::iterator it = obstacles.find(obstacleID);
	if( it != obstacles.end() )
	{
		delete obstacles[obstacleID];
		obstacles.erase(obstacleID);
	}
	else
	{
		qDebug() << "Could not delete obstacle" << obstacleID.c_str() << ": not found in obstacles map.";
	}
}


void satelliteView::simTargetMoveTo( double x, double y, double psi )
{
	std::lock_guard<std::mutex> lock(mpMutex);
	simTarget->setNewPosition(position{x,y,psi});
}

void satelliteView::simTargetClearTrajectory()
{
	std::lock_guard<std::mutex> lock(mpMutex);
	simTarget->clearTrajectory();
}

bool satelliteView::popMarkedPosition(position *pos)
{
	std::lock_guard<std::mutex> lock(mpMutex);
	if ( !MPs.empty() )
	{
		*pos = MPs.back()->getPos();
		delete MPs.back();
		MPs.pop_back();
		return true;
	}
	return false;
}

void satelliteView::markPosition( QMouseEvent *event){
	int xPixel = event->pos().x();
	int yPixel = event->pos().y();
	double x = svWidget->xAxis->pixelToCoord(xPixel);
	double y = svWidget->yAxis->pixelToCoord(yPixel);
	std::lock_guard<std::mutex> lock(mpMutex);
	MPs.push_back(new markedPosition(svWidget, x, y));
}

simObject::simObject(std::string targetID, QCustomPlot *satelliteViewWidget){
	ID = targetID;
	ownerSVWidget = satelliteViewWidget;
}

simObject::~simObject(){
	std::lock_guard<std::mutex> lock(positionMutex);
	ownerSVWidget->removePlottable(trajectory);
	ownerSVWidget->removePlottable(trajectoryHead);
	delete targetIcon;
}

void simObject::trajectoryInit()
{
	std::lock_guard<std::mutex> lock(positionMutex);
	trajectory = new QCPCurve(ownerSVWidget->xAxis, ownerSVWidget->yAxis);
	trajectory->setPen(QPen(Qt::blue));
	trajectoryHead = new QCPCurve(ownerSVWidget->xAxis, ownerSVWidget->yAxis);
}

void simObject::setNewPosition(position Pos)
{
	std::lock_guard<std::mutex> lock(positionMutex);
	pos = Pos;
}

position simObject::getPosition()
{
	std::lock_guard<std::mutex> lock(positionMutex);
	return pos;
}

void simObject::updateTrajectory(){
	std::lock_guard<std::mutex> lock(positionMutex);
	trajectory->addData(pos.x, pos.y);

	QMatrix rm;
	rm.rotate(pos.psi);
	QPixmap rotatedTargetIcon = targetIcon->transformed(rm);

	trajectoryHead->data()->clear();
	trajectoryHead->setScatterStyle(QCPScatterStyle(rotatedTargetIcon));
	trajectoryHead->addData(pos.x, pos.y);
}

void simObject::clearTrajectory()
{
	std::lock_guard<std::mutex> lock(positionMutex);
	trajectory->data()->clear();
}

USV::USV(std::string USVid, QCustomPlot *satelliteViewWidget, double X, double Y, double Psi) : simObject(USVid, satelliteViewWidget){
	position Pos = {X, Y, Psi};
	this->setNewPosition(Pos);
}

void USV::makePlottable()
{
	if( readyToPlot ) return;
	trajectoryInit();
	//targetIcon = new QPixmap(":/pics/OdinIcon.png");
	//targetIcon = new QPixmap();
	//targetIcon->load("OdinIcon.png");
	targetIcon = new QPixmap(10, 10);
	targetIcon->fill(Qt::blue);
	// QPainter p(targetIcon);
	// p.drawEllipse(QRect(0,0,15, 15));
	// p.fill(Qt::blue);
	readyToPlot = true;
}

obstacle::obstacle(std::string obstID, QCustomPlot *satelliteViewWidget, double X, double Y, double Psi) : simObject(obstID, satelliteViewWidget){
	position Pos = {X, Y, Psi};
	this->setNewPosition(Pos);
}

void obstacle::makePlottable()
{
	if( readyToPlot ) return;
	trajectoryInit();
	targetIcon = new QPixmap(20, 20);
	targetIcon->fill(Qt::green);
	this->trajectory->setVisible(false);
	readyToPlot = true;
}
