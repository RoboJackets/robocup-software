#include "Bool_Parameter.hpp"

#include <stdexcept>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

Tactics::Bool_Parameter::Bool_Parameter(Tactics::Base *tactic, const char *name, bool def): Parameter(tactic, name)
{
    _value = def;
    _default_value = def;
}

void Tactics::Bool_Parameter::clear()
{
    _valid = false;
    _value = _default_value;
}

void Tactics::Bool_Parameter::set(float v)
{
    if (v == 0)
    {
        _valid = true;
        _value = false;
    } else if (v == 1)
    {
        _valid = true;
        _value = true;
    } else {
        throw out_of_range(str(format("Parameter \"%1\" is boolean") % _name));
    }
}

void Tactics::Bool_Parameter::set(const string &value)
{
    if (value == "false")
    {
        _valid = true;
        _value = false;
    } else if (value == "true")
    {
        _valid = true;
        _value = true;
    } else {
        throw out_of_range(str(format("Parameter \"%1\" is boolean") % _name));
    }
}
