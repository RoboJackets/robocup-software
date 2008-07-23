#include <stdio.h>
#include <QCoreApplication>

#include <Team.h>

#include "Skills.hpp"

void usage(const char* prog)
{
	printf("Usage: %s [-b] [-y]\n", prog);
	printf("\t-b: enable control of blue team.\n");
	printf("\t-y: enable control of yellow team.\n");
}

int main(int argc, char* argv[])
{
	QCoreApplication a(argc, argv);
	
	bool startYellow = false, startBlue = false;
	
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
	}
		
	if (!startBlue && !startYellow)
	{
		usage(argv[0]);
		return -1;
	}
	
	Skills* blue = 0;
	if (startBlue)
	{
		blue = new Skills(Blue);
		blue->start();
	}
	
	Skills* yellow = 0;
	if (startYellow)
	{
		yellow = new Skills(Yellow);
		yellow->start();
	}
	
	return a.exec();
}
