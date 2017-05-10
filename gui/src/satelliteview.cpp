#include "satelliteview.h"
#include <math.h>

#include <QDebug>
#include <QVBoxLayout>
#include <QPixmap>
#include <QFile>


satelliteView::satelliteView(QCustomPlot *satelliteViewWidget)
{
	svWidget = satelliteViewWidget;

	svWidget->setBackground(Qt::lightGray);
	svWidget->axisRect()->setBackground(Qt::white);

	svWidget->axisRect()->setupFullAxesBox();
	svWidget->xAxis->setRange(-2, 2);
	svWidget->yAxis->setRange(-2, 2);

	// make left and bottom axes transfer their ranges to right and top axes:
	connect(svWidget->xAxis, SIGNAL(rangeChanged(QCPRange)), svWidget->xAxis2, SLOT(setRange(QCPRange)));
	connect(svWidget->yAxis, SIGNAL(rangeChanged(QCPRange)), svWidget->yAxis2, SLOT(setRange(QCPRange)));

	this->initialize_buttons();

	connect(&plotTimer, SIGNAL(timeout()), this, SLOT(updatePlot()));
	plotTimer.start(30); // Interval 0 means to refresh as fast as possible

	connect(svWidget, SIGNAL(mousePress(QMouseEvent*)), SLOT(markPosition( QMouseEvent *)));

	simTarget = new USV("USV_Odin", svWidget, gpsPointStamped(10.472, 59.438, 0));
	simTarget->makePlottable();
}

satelliteView::~satelliteView(){
	delete zoomInButton;
	delete zoomOutButton;
	delete rangeReference;
	delete rangeDescriptor;
	
	while ( !MPs.empty() )
	{
		delete MPs.back();
		MPs.pop_back();
	}

	delete simTarget;

	for( auto const& aso : activeSimObjects ){
		delete aso.second;
	}
}

void satelliteView::updatePlot()
{
	std::lock_guard<std::mutex> lock(m);

	simTarget->updateTrajectory();


	vector<string> outdatedSimObjects;
	for( auto const& aso : activeSimObjects )
	{
		if(!aso.second->isPlottable())
		{
			aso.second->makePlottable();	
		}

		// Check if outdated
		QTime now = QTime::currentTime();
		QTime lastUpdate = aso.second->getPosition().timeStamp;
		if(lastUpdate.msecsTo(now) > 200){
			outdatedSimObjects.push_back(aso.first);
			qDebug() << "push_back" << aso.first.c_str();
		}
		else{
			aso.second->updateTrajectory();
		}
	}

	for( auto const& ID : outdatedSimObjects){
		delete activeSimObjects[ID];
		activeSimObjects.erase(ID);
	}

	gpsPointStamped plotCenter = simTarget->getPosition();

	double xrange = mapRangeInMeters*longitude_degs_pr_meter(plotCenter.latitude);
	double yrange = mapRangeInMeters*latitude_degs_pr_meter();
	svWidget->xAxis->setRange(plotCenter.longitude - xrange/2, plotCenter.longitude + xrange/2);
	svWidget->yAxis->setRange(plotCenter.latitude - yrange/2, plotCenter.latitude + yrange/2);
	svWidget->replot();
}

void satelliteView::markPosition( QMouseEvent *event){
	int xPixel = event->pos().x();
	int yPixel = event->pos().y();
	double x = svWidget->xAxis->pixelToCoord(xPixel);
	double y = svWidget->yAxis->pixelToCoord(yPixel);
	std::lock_guard<std::mutex> lock(m);
	MPs.push_back(new markedPosition(svWidget, x, y));
}

void satelliteView::zoomIn()
{
	std::lock_guard<std::mutex> lock(m);
	if( mapRangeInMeters > 100 )
		mapRangeInMeters = mapRangeInMeters/2;
	update_range_descriptor();
}

void satelliteView::zoomOut()
{
	std::lock_guard<std::mutex> lock(m);
	if( mapRangeInMeters < 12800 )
		mapRangeInMeters = mapRangeInMeters*2;
	update_range_descriptor();
}

void satelliteView::initialize_buttons()
{
	zoomInButton = new QPushButton("+", svWidget);
	zoomInButton->move(100, 20);
	zoomInButton->setFixedSize(30, 30);
	zoomInButton->setStyleSheet("background-color: lightGray");
	connect(zoomInButton, SIGNAL (released()), this, SLOT (zoomIn()));

	zoomOutButton = new QPushButton("-", svWidget);
	zoomOutButton->move(135, 20);
	zoomOutButton->setFixedSize(30, 30);
	zoomOutButton->setStyleSheet("background-color: lightGray");
	connect(zoomOutButton, SIGNAL (released()), this, SLOT (zoomOut()));

    int rangeRefWidth = svWidget->rect().width()/10;
	rangeReference = new QFrame(svWidget);
    rangeReference->setObjectName(QString::fromUtf8("line"));
    rangeReference->setGeometry(QRect(190, 45, rangeRefWidth, 3));
    rangeReference->setFrameShape(QFrame::HLine);
    rangeReference->setFrameShadow(QFrame::Sunken);

	rangeDescriptor = new QLabel(svWidget);
	rangeDescriptor->move(190, 20);
	rangeDescriptor->setFixedSize(rangeRefWidth, 30);
	rangeDescriptor->setStyleSheet("background-color: transparent");
	rangeDescriptor->setAlignment(Qt::AlignCenter);
	update_range_descriptor();
}

void satelliteView::update_range_descriptor()
{
	rangeDescriptor->setText(QString::number(mapRangeInMeters/10) + "m");
}

void satelliteView::addSimObject( string objectID, string objectDescriptor, double x, double y, double psi )
{
	std::lock_guard<std::mutex> lock(m);

	map<string, simObject*>::iterator it = activeSimObjects.find(objectID);
	if( it != activeSimObjects.end() )
	{
		// This simObject already exists
		return;
	}
	gpsPointStamped pos(x, y, psi);
	if( objectDescriptor == "fixed_obstacle" )
	{
		qDebug() << "Added new fixed obstacle...";
		activeSimObjects[objectID] = new obstacle(objectID.c_str(), svWidget, pos);
	}
	else if( objectDescriptor == "ship")
	{
		qDebug() << "Added new ship...";
		activeSimObjects[objectID] = new ship(objectID.c_str(), svWidget, pos);
	}
}

void satelliteView::setPosition( string objectID, gpsPointStamped newPos )
{
	std::lock_guard<std::mutex> lock(m);
	map<string, simObject*>::iterator it = activeSimObjects.find(objectID);
	if( it != activeSimObjects.end() )
	{
		activeSimObjects[objectID]->setNewPosition(newPos);
	}else
	{
		//addSimObject(objectID, x, y, psi);
	}
}

void satelliteView::removeSimObject(string objectID)
{
	std::lock_guard<std::mutex> lock(m);

	map<string, simObject*>::iterator it = activeSimObjects.find(objectID);
	if( it != activeSimObjects.end() )
	{
		delete activeSimObjects[objectID];
		activeSimObjects.erase(objectID);
	}
	else
	{
		qDebug() << "Could not delete obstacle" << objectID.c_str() << ": not found in activeSimObjects map.";
	}
}

bool satelliteView::doesExist( string simObjectID )
{
	std::lock_guard<std::mutex> lock(m);
	map<string, simObject*>::iterator it = activeSimObjects.find(simObjectID);
	if( it != activeSimObjects.end() )
	{
		return true;
	}
	return false;
}


void satelliteView::simTargetMoveTo( gpsPointStamped pos )
{
	std::lock_guard<std::mutex> lock(m);
	simTarget->setNewPosition(pos);
}

void satelliteView::simTargetClearTrajectory()
{
	std::lock_guard<std::mutex> lock(m);
	simTarget->clearTrajectory();
}

bool satelliteView::popMarkedPosition(position *pos)
{
	std::lock_guard<std::mutex> lock(m);
	if ( !MPs.empty() )
	{
		*pos = MPs.back()->getPos();
		delete MPs.back();
		MPs.pop_back();
		return true;
	}
	return false;
}


simObject::simObject(std::string id, QCustomPlot *satelliteViewWidget){
	ID = id;
	ownerSVWidget = satelliteViewWidget;
}

simObject::~simObject(){
	std::lock_guard<std::mutex> lock(m);
	ownerSVWidget->removePlottable(trajectory);
	ownerSVWidget->removePlottable(trajectoryHead);
	delete targetIcon;
}

void simObject::trajectoryInit()
{
	std::lock_guard<std::mutex> lock(m);
	trajectory = new QCPCurve(ownerSVWidget->xAxis, ownerSVWidget->yAxis);
	trajectory->setPen(QPen(Qt::blue));
	trajectoryHead = new QCPCurve(ownerSVWidget->xAxis, ownerSVWidget->yAxis);
}

void simObject::setNewPosition(gpsPointStamped Pos)
{
	std::lock_guard<std::mutex> lock(m);
	pos = Pos;
}

gpsPointStamped simObject::getPosition()
{
	std::lock_guard<std::mutex> lock(m);
	return pos;
}

void simObject::updateTrajectory(){
	std::lock_guard<std::mutex> lock(m);
	trajectory->addData(pos.longitude, pos.latitude);

	QMatrix rm;
	rm.rotate(pos.heading);
	QPixmap rotatedTargetIcon = targetIcon->transformed(rm);

	trajectoryHead->data()->clear();
	trajectoryHead->setScatterStyle(QCPScatterStyle(rotatedTargetIcon));
	trajectoryHead->addData(pos.longitude, pos.latitude);
}

void simObject::clearTrajectory()
{
	std::lock_guard<std::mutex> lock(m);
	trajectory->data()->clear();
}

USV::USV(std::string USVid, QCustomPlot *satelliteViewWidget, gpsPointStamped Pos) : simObject(USVid, satelliteViewWidget){
	this->setNewPosition(Pos);
}

void USV::makePlottable()
{
	if( readyToPlot ) return;
	trajectoryInit();
	//targetIcon = new QPixmap(":/pics/OdinIcon.png");
	//targetIcon = new QPixmap();
	//targetIcon->load("OdinIcon.png");
	targetIcon = new QPixmap(10, 20);
	targetIcon->fill(Qt::black);
	// QPainter p(targetIcon);
	// p.drawEllipse(QRect(0,0,15, 15));
	// p.fill(Qt::blue);
	readyToPlot = true;
}

obstacle::obstacle(std::string obstID, QCustomPlot *satelliteViewWidget, gpsPointStamped Pos) : simObject(obstID, satelliteViewWidget){
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

ship::ship(std::string shipID, QCustomPlot *satelliteViewWidget, gpsPointStamped Pos) : simObject(shipID, satelliteViewWidget){
	this->setNewPosition( Pos );
}

void ship::makePlottable()
{
	if( readyToPlot ) return;
	trajectoryInit();
	targetIcon = new QPixmap(10, 30);
	targetIcon->fill(Qt::blue);
	this->trajectory->setVisible(true);
	readyToPlot = true;
}
