// #include "VisionGen.hpp"
#include "Viewer.hpp"
#include "Physics/Env.hpp"
#include "Config.hpp"
// #include "Radio.hpp"
// #include "CommandReceiver.hpp"

#include <QApplication>

#include <NxPhysics.h>
#include <stdio.h>

using namespace std;

void usage(const char* prog)
{
    printf("usage: %s -c <config file> [--ui] [--noisy]\n", prog);
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    Env* env = new Env();

    char* configFile = 0;
	bool useGUI = false;
	bool useNoisy = false;

    //loop arguments and look for config file
    for (int i=1 ; i<argc ; ++i)
    {
		if (strcmp(argv[i], "--ui") == 0)
		{
			useGUI = true;
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
        } else if (strcmp(argv[i], "--noisy") == 0) {
        	useNoisy = true;
        } else {
            printf("%s is not recognized as a valid flag\n", argv[i]);
            return 0;
        }
    }

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

//     CommandReceiver cmd(env);
//     cmd.start();

//     VisionGen vision0(env, 0, useNoisy);
//     vision0.start();
    
    //VisionGen vision1(env, 1); // old code for multiple cameras
    //vision1.start();

//     Radio radioBlue(Blue, *env);
//     Radio radioYellow(Yellow, *env);
//     radioBlue.start();
//     radioYellow.start();

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
//     vision0.terminate();
//     vision0.wait();
    
    //vision1.terminate();
    //vision1.wait();

//     radioBlue.terminate();
//     radioBlue.wait();
//     radioYellow.terminate();
//     radioYellow.wait();

    delete config;

    return ret;
}
