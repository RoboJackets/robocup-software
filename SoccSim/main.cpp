#include "Viewer.hpp"
#include "Physics/Env.hpp"
#include "Config.hpp"

#include <QApplication>

#include <NxPhysics.h>
#include <stdio.h>

using namespace std;

void usage(const char* prog)
{
	fprintf(stderr, "usage: %s -c <config file> [--ui] [--sv]\n", prog);
	fprintf(stderr, "\t--ui    Show GUI\n");
	fprintf(stderr, "\t--sv    Use shared vision multicast port\n");
}

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	Env* env = new Env();

	char* configFile = 0;
	bool useGUI = false;
	bool sendShared = false;

	//loop arguments and look for config file
	for (int i=1 ; i<argc ; ++i)
	{
		if (strcmp(argv[i], "--ui") == 0)
		{
			useGUI = true;
		} else if (strcmp(argv[i], "--sv") == 0)
		{
			sendShared = true;
		} else if (strcmp(argv[i], "-c") == 0)
		{
			++i;
			if (i < argc)
			{
				configFile = argv[i];
			}
			else
			{
				printf ("Expected config file after -c parameter\n");
				return 0;
			}
		} else {
			printf("%s is not recognized as a valid flag\n", argv[i]);
			return 0;
		}
	}

	env->sendShared = sendShared;
	
	Config* config = 0;

	if (configFile)
	{
		config = new Config(configFile, env);
	}
	else
	{
		usage(argv[0]);
		exit(0);
	}

	Viewer *win = 0;
	if (useGUI)
	{
		win = new Viewer(env);
		win->setVisible(true);
	}
	int ret = app.exec();

	//cleanup
	delete win;
	delete env;

	delete config;

	return ret;
}
