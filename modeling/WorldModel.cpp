#include "WorldModel.hpp"

#include <QObject>
#include <iostream>

WorldModel::WorldModel() :
	Module("World Model")
{
	
}

void WorldModel::run()
{
	SystemState::WorldModelOut& state = _state->worldModelOut;

}