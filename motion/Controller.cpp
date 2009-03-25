#include "Controller.hpp"

using namespace Motion;

Controller::Controller(ConfigFile::RobotCfg cfg, unsigned int id[]) :
	Module("Motion") /*_config(filename),*/
{
//     try
//     {
//         _config.load();
//     }
//     catch (std::runtime_error& re)
//     {
//         printf("Config Load Error: %s\n", re.what());
//     }

    for(unsigned int i=0 ; i<5 ; i++)
    {
        cfg.id = id[i];
        _robots[i] = new Robot(cfg);
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
