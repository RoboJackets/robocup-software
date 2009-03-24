#include "Controller.hpp"

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

    for(unsigned int i=0 ; i<5 ; i++)
    {
        _robots[i] = new Robot(_config.robotConfig(i));
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
    _config.setElement("linearController,kp",value);
    for(unsigned int i=0; i<5; i++)
    {
        _robots[i]->setKp(value);
    }
}

void Controller::setKdGains(double value)
{
    _config.setElement("linearController,kd",value);
    for(unsigned int i=0; i<5; i++)
    {
        _robots[i]->setKd(value);
    }
}

void Controller::saveGains(QString filename)
{
    _config.save(filename);
}
void Controller::run()
{
    for(unsigned int i=0; i<5; i++)
    {
        _robots[i]->setSystemState(_state);
        _robots[i]->proc();
    }
}
