#include <QApplication>
#include "Referee.hpp"

int main(int argc,char* argv[])
{
	
	GameControl gameControl;
	
	QApplication application(argc,argv);

	Referee *win;
	win = new Referee(gameControl);
	win->show();
	
	return application.exec();
	gameControl.close();
} 
