#include "Module.hpp"

Module::Module(std::string name) :
	_name(name)
{
	
}

void Module::setSystemState(SystemState* state)
{
	_state = state;
}
