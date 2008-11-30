#include "Motion.hpp"

#include "Robot.hpp"
#include "ConfigFile.hpp"
#include "Team.h"

#include <QString>

Motion::Motion(QString filename) :
    Module("Motion"), _config(filename)
{
    _config.load();
    for(unsigned int i=0 ; i<5 ; ++i)
    {
        _robots[i] = new Robot(_config.robotConfig(i));
    }
}

Motion::~Motion()
{
    for (unsigned int i=0 ; i<5 ; ++i)
    {
	delete _robots[i];
	_robots[i] = 0;
    }
}

void Motion::run()
{
    for(unsigned int i=0; i<5; ++i)
    {
        _robots[i]->proc();
    }

}
