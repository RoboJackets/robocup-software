#include "Module.hpp"

#include <stdexcept>

Module::Module(QString name)
// 	_name(name)
{
	_widget = 0;
	_selectedRobotId = -1;
	_toolbar = 0;
}
