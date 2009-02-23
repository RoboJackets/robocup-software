#include "Controller.hpp"

using namespace Motion;

//TODO add slot for controlling individual robots
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

    for(unsigned int i=0 ; i<5 ; i++)
    {
        _robots[i] = new Robot(_config.robotConfig(i));
    }
    printf("Motion is Running\n");
}

Controller::~Controller()
{
    for (unsigned int i=0 ; i<5 ; i++)
    {
    	delete _robots[i];
    	_robots[i] = 0;
    }
}

void Controller::run()
{
    for(unsigned int i=0; i<5; i++)
    {
	_robots[i]->setSystemState(_state);
	_robots[i]->proc();
    }
}
