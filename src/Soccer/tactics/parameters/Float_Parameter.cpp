#include "Float_Parameter.hpp"

Tactics::Float_Parameter::Float_Parameter(Tactics::Base *tactic, const char *name, float def): Parameter(tactic, name)
{
    _value = def;
    _default_value = def;
}

void Tactics::Float_Parameter::clear()
{
    _valid = false;
    _value = _default_value;
}

void Tactics::Float_Parameter::set(float value)
{
    _valid = true;
    _value = value;
}
