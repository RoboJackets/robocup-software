#include "Parameter.hpp"
#include "../Behavior.hpp"

#include <Utils.hpp>

#include <stdexcept>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

Gameplay::Parameter::Parameter(Behavior *behavior, const char *name)
{
    _valid = false;
    _name = name;
    _behavior = behavior;
    
    _behavior->_parameters[name] = this;
}

Gameplay::Parameter::~Parameter()
{
    Utils::map_remove(_behavior->_parameters, this);
}

void Gameplay::Parameter::clear()
{
    _valid = false;
}

void Gameplay::Parameter::set(float value)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept floats") % _name));
}

void Gameplay::Parameter::set(const Geometry2d::Point &point)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept points") % _name));
}

void Gameplay::Parameter::set(const Geometry2d::TransformMatrix *matrix, const Geometry2d::Point &point)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept points with matrix qualifiers") % _name));
}

void Gameplay::Parameter::set(Robot *robot)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept robots") % _name));
}

void Gameplay::Parameter::set(const string &value)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept strings") % _name));
}
