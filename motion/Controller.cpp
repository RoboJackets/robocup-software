#include "Controller.hpp"

using namespace Motion;

Controller::Controller(ConfigFile::RobotCfg cfg) :
	Module("Motion")
{
    for(unsigned int i=0 ; i<5 ; i++)
    {
        _robots[i] = new Robot(cfg, i);
    }

}

Controller::~Controller()
{
    for (unsigned int i=0; i<5; i++)
    {
        delete _robots[i];
        _robots[i] = 0;
    }
}

void Controller::setKpGains(double value)
{
    for(unsigned int i=0; i<5; i++)
    {
        _robots[i]->setKp(value);
    }
}

void Controller::setKdGains(double value)
{
    for(unsigned int i=0; i<5; i++)
    {
        _robots[i]->setKd(value);
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
