#include "Robot_Parameter.hpp"
#include "../../Robot.hpp"

Tactics::Robot_Parameter::Robot_Parameter(Base *tactic, const char *name): Parameter(tactic, name)
{
    _robot = 0;
}

void Tactics::Robot_Parameter::clear()
{
    _valid = false;
    _robot = 0;
}

void Tactics::Robot_Parameter::set(Robot *value)
{
    _valid = true;
    _robot = value;
}

bool Tactics::Robot_Parameter::valid() const
{
    return _robot && _robot->visible();
}
