#include <pthread.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "watchdog.h"
#include "process.h"
#include "comm.h"
#include "config.h"
#include "log.h"

using namespace std;

const char *config_file = "config.xml";

int original_argc;
char **original_argv;

int main(int argc, char *argv[])
{
    original_argc = argc;
    original_argv = argv;
    
    if (argc > 1)
    {
        config_file = argv[1];
    }
    
    mkdir("/tmp/watchdog", 0777);
    mkdir("/tmp/watchdog/latest", 0777);

    log_open("watchdog");
    unlink("/tmp/watchdog/latest/watchdog-latest.log");
    symlink(log_filename.c_str(), "/tmp/watchdog/latest/watchdog-latest.log");
    
    log_printf("--- Watchdog starting ---\n");
    log_printf("Loading configuration from %s\n", config_file);
    
    // Get the default DISPLAY value
    char *value = getenv("DISPLAY");
    if (value)
    {
        Process::default_display = value;
    } else {
        Process::default_display.clear();
    }

    config_load(config_file);
    
    Connection::start_server();
    
    // Start all processes that were enabled in the config file
    Process::lock();
    for (list<Process *>::const_iterator i = Process::process_list.begin(); i != Process::process_list.end(); ++i)
    {
        Process *process = *i;
        
        if (process->enabled)
        {
            process->start();
        }
    }
    Process::unlock();
    
    Process::monitor_loop();
    
    return 0;
}
