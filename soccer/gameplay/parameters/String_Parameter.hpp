#pragma once

#include "Parameter.hpp"

#include <vector>

namespace Gameplay
{    
	class String_Parameter: public Parameter
	{
	public:
		String_Parameter(Behavior *behavior, const char *name, const std::string &def = std::string());
		
		virtual void clear();
		virtual void set(const std::string &value);
		
		const std::string &value() const { return _value; }
		
		std::vector<std::string> legal;
		
	protected:
		std::string _value;
		std::string _default_value;
	};
}
