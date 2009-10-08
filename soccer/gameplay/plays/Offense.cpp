#include "Offense.hpp"

Gameplay::Plays::Offense::Offense(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::Offense::applicable()
{
	//FIXME - Offense/defense
	return _gameplay->state()->gameState.playing();
}

bool Gameplay::Plays::Offense::run()
{
	return true;
}
