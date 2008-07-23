#include "Parameter.hpp"
#include "../Tactics.hpp"
#include "../../map_utils.hpp"

#include <stdexcept>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

Tactics::Parameter::Parameter(Tactics::Base *tactic, const char *name)
{
    _valid = false;
    _name = name;
    _tactic = tactic;
    
    _tactic->_parameters[name] = this;
}

Tactics::Parameter::~Parameter()
{
    map_remove(_tactic->_parameters, this);
}

void Tactics::Parameter::clear()
{
    _valid = false;
}

void Tactics::Parameter::set(float value)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept floats") % _name));
}

void Tactics::Parameter::set(const Geometry::Point2d &point)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept points") % _name));
}

void Tactics::Parameter::set(const Geometry::TransformMatrix *matrix, const Geometry::Point2d &point)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept points with matrix qualifiers") % _name));
}

void Tactics::Parameter::set(Robot *robot)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept robots") % _name));
}

void Tactics::Parameter::set(const string &value)
{
    throw invalid_argument(str(format("Parameter \"%1%\" does not accept strings") % _name));
}
