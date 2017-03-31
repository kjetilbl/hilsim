#ifndef REALTIMEPLOT_H
#define REALTIMEPLOT_H

#include "../qcustomplot.h"
#include <QTimer>
#include <QObject>

using namespace std;

class realtimePlot : public QObject
{
	Q_OBJECT
public:
	realtimePlot(QCustomPlot *plotWidget);
	void title(const QString plotTitle);
	void ylabel(const QString label);
	void updateValues(double measured, double ref);
	void clear();

private slots:
	void updateRtPlot();

private:
	QCustomPlot *rtPlotWidget;
	QTimer plotTimer;
	double measuredValue, refValue;
};

#endif // REALTIMEPLOT_H
