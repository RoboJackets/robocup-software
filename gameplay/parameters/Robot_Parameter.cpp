#include "Robot_Parameter.hpp"
#include "../Robot.hpp"

Gameplay::Robot_Parameter::Robot_Parameter(Behavior *behavior, const char *name): Parameter(behavior, name)
{
    _robot = 0;
}

void Gameplay::Robot_Parameter::clear()
{
    _valid = false;
    _robot = 0;
}

void Gameplay::Robot_Parameter::set(Robot *value)
{
    _valid = true;
    _robot = value;
}

bool Gameplay::Robot_Parameter::valid() const
{
    return _robot && _robot->visible();
}
