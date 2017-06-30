#ifndef REALTIMEPLOT_H
#define REALTIMEPLOT_H

#include "../qcustomplot.h"
#include <QTimer>
#include <QObject>

using namespace std;




// TODO: legge inn mutex her, aksesseres både av posupdateHandler og timeout slot.
class realtimePlot : public QObject
{
	Q_OBJECT
public:
	realtimePlot(QCustomPlot *plotWidget);
	~realtimePlot(){};
	void title(const QString plotTitle);
	void ylabel(const QString label);
	void updateValues(double measured);
	void clear();

private slots:
	void updateRtPlot();

private:
	QCustomPlot *rtPlotWidget;
	QTimer plotTimer;
	double measuredValue;
};

#endif // REALTIMEPLOT_H
