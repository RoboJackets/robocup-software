// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <iostream>
#include "StateIDModule.hpp"

using namespace std;
using namespace StateIdentification;

StateIDModule::StateIDModule(SystemState *state) : Module("State ID")
{
	_state = state;
}

StateIDModule::~StateIDModule()
{
	
}

void StateIDModule::run()
{
	//TODO: insert code to process the filtered state here
}