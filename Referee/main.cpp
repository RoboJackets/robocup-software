#include <QApplication>

#include "Referee.hpp"

int main(int argc,char* argv[])
{
	QApplication application(argc,argv);

	Referee win;
	win.show();
	
	
	return application.exec();
} 
