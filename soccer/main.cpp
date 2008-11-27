#include <QApplication>
#include <stdio.h>
#include <string.h>

#include <Team.h>

#include "MainWindow.hpp"

void usage(const char* prog)
{
	printf("usage: %s <-y|-b> [-c <config file>]\n", prog);
	printf("\t-y: run as the yellow team\n");
	printf("\t-b: run as the blue team\n");
}

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
		printf("Error: No team specified\n");
		usage(argv[0]);
		return 0;
	}
	
	MainWindow win(team);
	win.showMaximized();
	
	return app.exec();
}
