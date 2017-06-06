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

    ros::init(argc, argv, "gui");
	ros::NodeHandle nh;

    MainWindow *w = new MainWindow(&nh);
    w->show();

    signal(SIGINT, quit_application);
	
	ros::AsyncSpinner spinner(1);
	spinner.start();
    int exitCode = a.exec();

    w->close();    
    delete w;

    qDebug() << "GUI finished with exit code" << exitCode;

    return exitCode;
}
