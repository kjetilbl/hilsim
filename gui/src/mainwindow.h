#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "../qcustomplot.h"
#include "realtimeplot.h"
#include "satelliteview.h"
#include "obstacleinterface.h"
#include "posUpdateHandler.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char *argv[], QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
	realtimePlot *headingPlot;
	realtimePlot *velocityPlot;
	satelliteView *sv;
	obstacleInterface *obstInterface;
	posUpdateHandler *puh;
	QThread *puhThread;
};

#endif // MAINWINDOW_H