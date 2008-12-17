#include <RadioTx.hpp>

#include "Controller.hpp"
#include "Gamepad.hpp"

using namespace Motion;

Controller::Controller(QString filename) :
	Module("Motion"), _config(filename)
{
    try
    {
	    _config.load();
    }
    catch (std::runtime_error& re)
    {
	    printf("Config Load Error: %s\n", re.what());
    }

    try
    {
	_gamepad = new Gamepad("/dev/input/js0");
	_mode = MANUAL;
    }
    catch(std::runtime_error& re)
    {
	printf("Running in autonmous. Must restart to run in manual\n");
    }

    for(unsigned int i=0 ; i<1 ; ++i)
    {
        _robots[i] = new Robot(_config.robotConfig(i));
    }
    printf("Motion is Running\n");
}

Controller::~Controller()
{
    for (unsigned int i=0 ; i<1 ; ++i)
    {
    	delete _robots[i];
    	_robots[i] = 0;
    }
}

void Controller::run()
{
    if(_mode == AUTO)
    {
        for(unsigned int i=0; i<1; ++i)
        {
            _robots[i]->setSystemState(_state);
            _robots[i]->proc();
        }
    }
    else
    {
        if (_gamepad->waitForInput(10))
        {
            if (_gamepad->b3())
	    {
                _mode = AUTO;
                printf("Switching to AUTO\n");
            }
        }

    }


}
