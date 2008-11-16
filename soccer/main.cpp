#include <QApplication>
#include <stdio.h>
#include <string.h>

#include <Team.h>

#include <JogDial.hpp>
#include <LogFile.hpp>
#include <LogFrame.hpp>

#include "MainWindow.hpp"

int main (int argc, char* argv[])
{
	QApplication app(argc, argv);
	
	Team team = UnknownTeam;
	
	for (int i=0 ; i<argc; ++i)
	{
		const char* var = argv[i];
		
		if (strcmp(var, "-y") == 0)
		{
			team = Yellow;
		}
		else if (strcmp(var, "-b") == 0)
		{
			team = Blue;
		}
	}
	
	if (team == UnknownTeam)
	{
		printf("Can't have an unknown team\n");
		return 0;
	}
	
	MainWindow win(team);
	win.showMaximized();
	
	return app.exec();
}
