#include <stdio.h>
#include <QApplication>

#include <LogFile.hpp>

#include "MainWindow.hpp"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	
	Log::LogFile* logfile = 0;
	if (argc == 2)
	{
		logfile = new Log::LogFile(argv[1]);
	}
	
	MainWindow win(logfile);
	win.showMaximized();
	
	int ret = app.exec();
	
	if (logfile)
	{
		delete logfile;
	}
	
	return ret;
}
