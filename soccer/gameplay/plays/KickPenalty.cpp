#include "KickPenalty.hpp"

Gameplay::Plays::KickPenalty::KickPenalty(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::KickPenalty::applicable()
{
	return gameState().setupRestart() && gameState().ourPenalty();
}

bool Gameplay::Plays::KickPenalty::run()
{
	return true;
}
