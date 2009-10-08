#include "Stopped.hpp"
#include "../GameplayModule.hpp"

#include <gameplay/behaviors/Idle.hpp>

Gameplay::Plays::Stopped::Stopped(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::Stopped::applicable()
{
	return _gameplay->state()->gameState.stopped();
}

bool Gameplay::Plays::Stopped::run()
{
	//FIXME
	self(1)->move(Geometry2d::Point(1, 1));
	
	return true;
}
