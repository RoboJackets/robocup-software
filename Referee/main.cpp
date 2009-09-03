#include <QApplication>
#include "Referee.hpp"

int main(int argc,char* argv[])
{
	std::string configFileName = "referee.conf";
	std::string logFileName = "gamecontrol.log";
	bool restart = false;
		
	GameControl gameControl;
	
	// load everything
	if (!gameControl.init(configFileName.c_str(), logFileName.c_str(), restart)) {
	     fprintf(stderr, "ERROR: Cannot intialize game controller\n");
	     return (1);
	}
	
	QApplication application(argc,argv);

	Referee *win;
	win = new Referee(gameControl);
	win->show();
	
	return application.exec();
	gameControl.close();
} 
