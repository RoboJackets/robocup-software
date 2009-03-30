#include "Module.hpp"

#include <stdexcept>

Module::Module(QString name) :
	_name(name)
{
    _state = 0;
	_widget = 0;
}

void Module::setSystemState(SystemState* state)
{
	_state = state;
}
