#ifndef SATELLITEVIEW_H
#define SATELLITEVIEW_H

#include "../qcustomplot.h"
#include <QTimer>
#include <QObject>
#include <mutex>
#include <string>
#include <map>

using namespace std;


class target;
class USV;
class obstacle;

struct position {
	double x;
	double y;
	double psi;
};

class markedPosition
{
public:
	markedPosition(const markedPosition& other) : x(other.x), y(other.y) {
		positionMark = other.positionMark;
		owner = other.owner;
	}
	markedPosition(QCustomPlot *ownerPlot, double X, double Y) : x(X), y(Y), owner(ownerPlot) {
		positionMark = new QCPCurve(owner->xAxis, owner->yAxis);
		positionMark->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle));
		positionMark->addData(x,y);
	}
	~markedPosition(){
		owner->removePlottable(positionMark);
	}
	position getPos(){
		return position{x,y,0};
	} 

private:
	double x;
	double y;
	QCPCurve *positionMark;
	QCustomPlot *owner;
};

class satelliteView : public QObject
{
	Q_OBJECT
public:
	satelliteView( QCustomPlot *satelliteViewWidget );
	void addObstacle( string obstacleID, double x, double y, double psi = 0 );
	void setPosition( string obstacleID, double x, double y, double psi = 0 );
	void deleteObstacle(string obstacleID);
	void simTargetMoveTo( double x, double y, double psi = 0 );
	void simTargetClearTrajectory();
	bool popMarkedPosition(position *pos);

private slots:
	void updatePlot();
	void markPosition( QMouseEvent *event );

private:
	mutex mpMutex;
	vector<markedPosition*> MPs;
	USV *simTarget;
	map<string, obstacle*> obstacles;
	QCustomPlot *svWidget;
	QTimer plotTimer;
};


//--------------------------------------------------------




class simObject // simObject
{
public:
	simObject(string targetID, QCustomPlot *satelliteViewWidget);
	~simObject();
	void 			setNewPosition(position Pos);
	void 			updateTrajectory();
	string 			getID(){return ID;};
	bool 			isPlottable(){ return readyToPlot; };
	virtual void 	makePlottable() = 0;
	position 		getPosition();
	void			clearTrajectory();

protected:
	void trajectoryInit();
	bool readyToPlot = false;
	mutex positionMutex;
	string ID;
	position pos;
	QCustomPlot *ownerSVWidget;
	QPixmap *targetIcon;
	QCPCurve *trajectory;
	QCPCurve *trajectoryHead;
};

class USV : public simObject
{
public:
	USV(string USVid, QCustomPlot *satelliteViewWidget, double X = 0, double Y = 0, double Psi = 0);
	void makePlottable();
};

class obstacle : public simObject
{
public:
	obstacle(string obstID, QCustomPlot *satelliteViewWidget, double X = 0, double Y = 0, double Psi = 0);
	void makePlottable();
};

#endif // SATELLITEVIEW_H
