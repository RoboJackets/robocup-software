#include "Float_Parameter.hpp"

Gameplay::Float_Parameter::Float_Parameter(Behavior *behavior, const char *name, float def): Parameter(behavior, name)
{
    _value = def;
    _default_value = def;
}

void Gameplay::Float_Parameter::clear()
{
    _valid = false;
    _value = _default_value;
}

void Gameplay::Float_Parameter::set(float value)
{
    _valid = true;
    _value = value;
}
