#include "Module.hpp"

#include <stdexcept>

std::map<std::string, Module*> Module::_modules;

Module::Module(std::string name) :
	_name(name)
{
	if (_modules[_name])
	{
		throw std::runtime_error("Module name conflict");
	}
	
	_modules[_name] = this; 
}

void Module::setSystemState(SystemState* state)
{
	_state = state;
}

Module* Module::module(std::string name)
{
	return _modules[name];
}
