#include "TheirKickoff.hpp"

Gameplay::Plays::TheirKickoff::TheirKickoff(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::TheirKickoff::applicable()
{
	return gameState().setupRestart() && gameState().theirKickoff();
}

bool Gameplay::Plays::TheirKickoff::run()
{
	return true;
}
