#include <QApplication>

#include "MainWindow.hpp"

int main(int argc,char* argv[])
{
	QApplication application(argc,argv);

	MainWindow win;
	win.show();
	
	return application.exec();
} 
