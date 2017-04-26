#include <src/mainwindow.h>
#include "/usr/include/qt4/Qt/qapplication.h"
#include <signal.h>

static void quit_application(int sig)
{
	qApp->quit();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow *w = new MainWindow(argc, argv);
    w->show();

    signal(SIGINT, quit_application);

    int exitCode = a.exec();

    w->close();    
    delete w;

    qDebug() << "GUI finished with exit code" << exitCode;

    return exitCode;
}
