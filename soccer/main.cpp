#include <mcheck.h>
#include <QApplication>
#include <stdio.h>
#include <string.h>
#include <QString>
#include <QDebug>

#include <Team.h>

#include <unistd.h>
#include <fcntl.h>

#include "MainWindow.hpp"

void usage(const char* prog)
{
	printf("usage: %s <-y|-b> [-c <config file>]\n", prog);
	printf("\t-y: run as the yellow team\n");
	printf("\t-b: run as the blue team\n");
    exit(1);
}

int main (int argc, char* argv[])
{
    mcheck(0);
    
    // Seed the large random number generator
    long int seed = 0;
    int fd = open("/dev/random", O_RDONLY);
    if (fd)
    {
        if (read(fd, &seed, sizeof(seed)) == sizeof(seed))
        {
            printf("seed %016lx\n", seed);
            srand48(seed);
        } else {
            fprintf(stderr, "Can't read /dev/random: %m\n");
        }
        close(fd);
    } else {
        fprintf(stderr, "Can't open /dev/random: %m\n");
    }
    
	QApplication app(argc, argv);

	Team team = UnknownTeam;
    QString cfgFile = "";

	for (int i=1 ; i<argc; ++i)
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
	    else if(strcmp(var, "-c") == 0)
	    {
            if (i+1 >= argc)
            {
                printf("no config file specified after -c");
                usage(argv[0]);
            }
            
            i++;
	    	cfgFile = argv[i];
	    }
        else
        {
            printf("Not a valid flag: %s\n", argv[i]);
            usage(argv[0]);
        }
	}

	if (team == UnknownTeam)
	{
		printf("Error: No team specified\n");
		usage(argv[0]);
		return 0;
	}
    
	MainWindow win(team, cfgFile);
	win.showMaximized();

	return app.exec();
}
