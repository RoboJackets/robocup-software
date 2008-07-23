#include <QCoreApplication>

#include "ConfigFile.hpp"
#include "MotionController.hpp"

using namespace Packet;

void usage(const char* prog)
{
	printf("Usage: %s [-b] [-y] [-a] <config file>\n", prog);
	printf("\t-b: enable blue team control.\n");
	printf("\t-y: enable yellow team control.\n");
	printf("\t-a: default to auto mode when a controller is plugged in.\n");
}

int main(int argc, char *argv[])
{
	QCoreApplication a(argc, argv);

	bool startYellow = false, startBlue = false;
	bool autoOn = false;
	char* cfgFile = 0;
	
	int c = 0;
	while (++c < argc)
	{
		if (strncmp(argv[c], "-b", 2) == 0)
		{
			startBlue = true;
		}
		else if (strncmp(argv[c], "-y", 2) == 0)
		{
			startYellow = true;
		}
		else if (strncmp(argv[c], "-a", 2) == 0)
		{
			//override manual default
			autoOn = true;
		}
		else
		{
			if (!cfgFile)
			{
				cfgFile = argv[c];
			}
		}
	}
	
	if (!cfgFile)
	{
		usage(argv[0]);
		return 0;
	}

	ConfigFile cfg(cfgFile);
	try
	{
		cfg.load();
	}
	catch (std::runtime_error& re)
	{
		printf("Config Load Error: %s\n", re.what());
		return 0;
	}
	
	MotionController* yellow = 0;
	if (startYellow)
	{
		yellow = new MotionController(Yellow, cfg, autoOn);
		yellow->start();
	}
	
	MotionController* blue = 0;
	if (startBlue)
	{
		blue = new MotionController(Blue, cfg, autoOn);
		blue->start();
	}
	
	return a.exec();
}
