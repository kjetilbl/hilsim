#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <QTimer>
#include <QObject>
#include <QDebug>
#include <QThread>

class timerTest : public QThread
{
	Q_OBJECT
public:
	timerTest(QThread *parent = 0) : QThread(parent){

	}

private slots:
	void timoutHandler(){
		qDebug("zUP2?");
	}

private:
	QTimer *timer;
	void run(){
		timer = new QTimer(this);
		qDebug() << "Connect:" << connect(timer, SIGNAL(timeout()), this, SLOT(timoutHandler()) );
		timer->start(500);
		qDebug() << "timer active:" << timer->isActive();
		qDebug() << "timerTest alive!";
		qDebug() << "timerTest running from thread " << QThread::currentThreadId();
		QThread::exec();
	}
};

#endif // WATCHDOG_H