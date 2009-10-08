#include "OurKickoff.hpp"

Gameplay::Plays::OurKickoff::OurKickoff(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::OurKickoff::applicable()
{
	return gameState().setupRestart() && gameState().ourKickoff();
}

bool Gameplay::Plays::OurKickoff::run()
{
	return true;
}
