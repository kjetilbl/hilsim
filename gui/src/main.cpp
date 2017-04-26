#include <src/mainwindow.h>
#include "/usr/include/qt4/Qt/qapplication.h"
#include <signal.h>

static void quit_application(int sig)
{
    qDebug() << "\n---------------------------------------";
	qDebug() << "Quit GUI Application";
	qApp->quit();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow *w = new MainWindow(argc, argv);
    w->show();
    qDebug() << "GUI running..";

    signal(SIGINT, quit_application);

    int exitCode = a.exec();

    w->close();    
    delete w;

    qDebug() << "GUI finished with exit code" << exitCode;
    qDebug() << "---------------------------------------";

    return exitCode;
}
