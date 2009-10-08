#include "String_Parameter.hpp"

#include <boost/foreach.hpp>

Gameplay::String_Parameter::String_Parameter(Behavior *behavior, const char *name, const std::string &def)
	: Parameter(behavior, name)
{
	_value = def;
	_default_value = def;
}

void Gameplay::String_Parameter::clear()
{
	_valid = false;
	_value = _default_value;
}

void Gameplay::String_Parameter::set(const std::string &value)
{
	if (!legal.empty())
	{
		BOOST_FOREACH(const std::string &str, legal)
		{
			if (str == value)
			{
				_valid = true;
				_value = value;
				return;
			}
		}
		
		fprintf(stderr, "String_Parameter: invalid value \"%s\"\n", value.c_str());
		clear();
	} else {
		_valid = true;
		_value = value;
	}
}
