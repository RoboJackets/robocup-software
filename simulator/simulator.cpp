#include "physics/Environment.hpp"
#include "SimulatorWindow.hpp"
#include "SimulatorGLUTThread.hpp"

#include <QApplication>
#include <QFile>
#include <QThread>

#include <stdio.h>
#include <signal.h>

using namespace std;

void quit(int signal)
{
	fprintf(stderr, "Exiting due to signal %d\n", signal);
	exit(0);
}

void usage(const char* prog)
{
	fprintf(stderr, "usage: %s [-c <config file>] [--glut] [--sv]\n", prog);
	fprintf(stderr, "\t--help  Show usage message\n");
	fprintf(stderr, "\t--sv    Use shared vision multicast port\n");
}

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);

	QString configFile = "simulator.cfg";
	bool sendShared = false;

	//loop arguments and look for config file
	for (int i=1 ; i<argc ; ++i)
	{
		if (strcmp(argv[i], "--help") == 0)
		{
			usage(argv[0]);
			return 1;
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
				return 1;
			}
		} else if ((strcmp(argv[i], "--h") == 0) || (strcmp(argv[i], "--help") == 0))
		{
			usage(argv[0]);
			return 0;
		} else {
			printf("%s is not recognized as a valid flag\n", argv[i]);
			return 1;
		}
	}

	// create the thread for simulation
	SimulatorGLUTThread sim_thread(argc, argv, configFile, sendShared);

	struct sigaction act;
	memset(&act, 0, sizeof(act));
	act.sa_handler = quit;
	sigaction(SIGINT, &act, 0);

	// Create and initialize GUI with environment information
	SimulatorWindow win(sim_thread.env());
	win.show();

	// initialize socket connections separately
	sim_thread.env()->connectSockets();

	// start up threads
	sim_thread.start();
	int ret = app.exec();

	// shut down sim_thread
	sim_thread.wait();
//	sim_thread.exit(0);

	return 0;//ret;
}
