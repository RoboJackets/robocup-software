#include "DefendPenalty.hpp"

Gameplay::Plays::DefendPenalty::DefendPenalty(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::DefendPenalty::applicable()
{
	return gameState().setupRestart() && gameState().theirPenalty();
}

bool Gameplay::Plays::DefendPenalty::run()
{
	return true;
}
