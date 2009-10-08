#include "Defense.hpp"

Gameplay::Plays::Defense::Defense(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::Defense::applicable()
{
	//FIXME - Offense/defense
	return gameState().playing();
}

bool Gameplay::Plays::Defense::run()
{
	return true;
}
