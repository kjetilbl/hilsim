#include "realtimeplot.h"
#include <QDebug>

realtimePlot::realtimePlot(QCustomPlot *plotWidget){
	rtPlotWidget = plotWidget;

	rtPlotWidget->setBackground(Qt::lightGray);
	rtPlotWidget->axisRect()->setBackground(Qt::white);

	rtPlotWidget->addGraph(); // blue line
	rtPlotWidget->graph(0)->setPen(QPen(QColor(40, 110, 255)));

	QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
	timeTicker->setTimeFormat("%h:%m:%s");
	rtPlotWidget->xAxis->setTicker(timeTicker);
	rtPlotWidget->axisRect()->setupFullAxesBox();
	rtPlotWidget->yAxis->setRange(-1.2, 1.2);

	// make left and bottom axes transfer their ranges to right and top axes:
	connect(rtPlotWidget->xAxis, SIGNAL(rangeChanged(QCPRange)), rtPlotWidget->xAxis2, SLOT(setRange(QCPRange)));
	connect(rtPlotWidget->yAxis, SIGNAL(rangeChanged(QCPRange)), rtPlotWidget->yAxis2, SLOT(setRange(QCPRange)));

	connect(&plotTimer, SIGNAL(timeout()), this, SLOT(updateRtPlot()));
	plotTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void realtimePlot::updateValues(double measured){
	measuredValue = measured;
}

void realtimePlot::updateRtPlot()
{
	static QTime time(QTime::currentTime());
	// calculate two new data points:
	double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
	static double lastPointKey = 0;
	if (key-lastPointKey > 0.002) // at most add point every 2 ms
	{
	// add data to lines:
	rtPlotWidget->graph(0)->addData(key, measuredValue);
	// rescale value (vertical) axis to fit the current data:
	rtPlotWidget->graph(0)->rescaleValueAxis();
	lastPointKey = key;
	}
	// make key axis range scroll with the data (at a constant range size of 8):
	rtPlotWidget->xAxis->setRange(key, 8, Qt::AlignRight);
	rtPlotWidget->replot();
}

void realtimePlot::title(const QString plotTitle){
	rtPlotWidget->plotLayout()->insertRow(0);
	rtPlotWidget->plotLayout()->addElement(0, 0, new QCPTextElement(rtPlotWidget, plotTitle, QFont("sans", 12, QFont::Bold)));

}

void realtimePlot::ylabel(const QString label){
	rtPlotWidget->yAxis->setLabel(label);
}

void realtimePlot::clear()
{
	rtPlotWidget->graph(0)->data()->clear();
}