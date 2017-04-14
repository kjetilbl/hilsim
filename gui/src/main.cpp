#include <src/mainwindow.h>
#include "/usr/include/qt4/Qt/qapplication.h"

// Test

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w(argc, argv);
    w.show();
    qDebug() << "GUI running..";

    return a.exec();
}
