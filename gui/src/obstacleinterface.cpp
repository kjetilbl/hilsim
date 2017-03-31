#include <QVBoxLayout>
#include "obstacleinterface.h"

using namespace std;

environment::obstacleCmd makeObstacleCommand(string CmdSpecifier, double x = 0, double y = 0, double psi = 0, string ID = "no_id")
{
	environment::obstacleCmd newCommand;
	newCommand.cmdSpecifier = CmdSpecifier;
	newCommand.receiverID = ID;
	newCommand.x = x;
	newCommand.y = y;
	newCommand.psi = psi;
	return newCommand;
}

//----------------------------------------------------------------------------------------
//---------------------------------obstacleInterface--------------------------------------
//----------------------------------------------------------------------------------------



obstacleInterface::obstacleInterface(ros::NodeHandle nh, QGroupBox *interfaceWindow, satelliteView *Sv) 
	: sv(Sv)
{
	cmdPub = nh.advertise<environment::obstacleCmd>("obstacleCommandTopic", 1000);

	spawnObstacleButton = new QPushButton("Spawn Obstacles");
	QVBoxLayout *vbox = new QVBoxLayout;
	vbox->addWidget(spawnObstacleButton);
	interfaceWindow->setLayout(vbox);
	connect(spawnObstacleButton, SIGNAL (released()), this, SLOT (handleSpawnButton()));


}

void obstacleInterface::requestNewObstacle(double x, double y, double psi)
{
	environment::obstacleCmd newCommand = makeObstacleCommand("spawn", x, y, psi);
	cmdPub.publish(newCommand);
}

void obstacleInterface::handleSpawnButton()
{
	position newObstPos;
	while( sv->popMarkedPosition( &newObstPos ) == true )
	{
		requestNewObstacle(newObstPos.x, newObstPos.y, 45);
	}
}



