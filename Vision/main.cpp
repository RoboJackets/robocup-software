#include "Camera_Thread.h"
#include "Config_File.h"
#include "Camera_Window.h"
#include "vision/Process.h"
#include "vision/Sender.h"
#include "main.hpp"

#include <QApplication>
#include <signal.h>
#include <stdexcept>
#include <GL/glut.h>
#include <boost/foreach.hpp>

using namespace std;
using namespace boost;

bool debug = false;

void interrupted(int signal)
{
    printf("Received signal %d\n", signal);
    Camera_Thread::stop_all();
    exit(0);
}

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		printf("Name one or more configuration files on the command line.\n");
		return 1;
	}
	
	// Library initialization
    glutInit(&argc, argv);
    QApplication *app = new QApplication(argc, argv);

    // Signal handlers
    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = interrupted;
    sigaction(SIGINT, &act, 0);
    sigaction(SIGSEGV, &act, 0);
    sigaction(SIGABRT, &act, 0);

    // Load config files
    list<Config_File *> configs;
    for (int i = 1; i < argc; ++i)
    {
        if (!strcmp(argv[i], "-debug"))
        {
            debug = true;
        } else {
            try
            {
                configs.push_back(new Config_File(argv[i]));
            } catch (exception &ex)
            {
                printf("Error loading configuration %s: %s\n", argv[i], ex.what());
            }
        }
    }
    
    // Bail if we couldn't set up any cameras
    if (Camera_Thread::camera_threads().empty())
    {
    	printf("No cameras configured\n");
    	return 1;
    }
    
    // Set up the sender
    Vision::Process::sender = new Vision::Sender();
    
    // Main loop
    app->exec();
    
    // Exit normally
    Camera_Thread::stop_all();
    
    BOOST_FOREACH(Config_File *config, configs)
    {
    	delete config->window();
    }
    
    printf("\n");

    return 0;
}
