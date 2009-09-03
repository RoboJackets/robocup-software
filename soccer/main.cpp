// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <QApplication>
#include <QString>
#include <QDebug>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <Team.h>

#include "MainWindow.hpp"

using namespace std;

void usage(const char* prog)
{
	printf("usage: %s <-y|-b> [-ng] [-p <play dir>] [-f] [-c <config file>]\n", prog);
	printf("\t-y:  run as the yellow team\n");
	printf("\t-b:  run as the blue team\n");
	printf("\t-ng: no goalie\n");
	printf("\t-p:  specify the plays directory\n");
	printf("\t-c:  specify the configuration file\n");
	printf("\t-f:  flip field\n");
    exit(1);
}

int main (int argc, char* argv[])
{
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
    vector<const char *> playDirs;

    bool useGoalie = true;
    bool flip = false;
	for (int i=1 ; i<argc; ++i)
	{
	    const char* var = argv[i];

        if (strcmp(var, "-ng") == 0)
        {
            useGoalie = false;
        } else if (strcmp(var, "-p") == 0 && i < (argc - 1))
        {
            playDirs.push_back(argv[i + 1]);
			++i;
        } else if (strcmp(var, "-y") == 0)
	    {
	    	team = Yellow;
	    }
	    else if (strcmp(var, "-b") == 0)
	    {
	    	team = Blue;
	    }
	    else if (strcmp(var, "-f") == 0)
		{
	    	flip = true;
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
    
    if (useGoalie)
    {
        win.processor()->gameplayModule()->createGoalie();
    }
    
    win.processor()->flip_field(flip);
	
	BOOST_FOREACH(const char *dir, playDirs)
	{
		boost::filesystem::path p(dir);
	    if (is_regular(p))
	    {
	    	win.processor()->gameplayModule()->loadPlay(dir);
	    }
	    else
	    {
	    	win.processor()->gameplayModule()->loadPlays(dir);
	    }
	}
    
	win.showMaximized();

	return app.exec();
}
