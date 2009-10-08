#include "TheirFreekick.hpp"

Gameplay::Plays::TheirFreekick::TheirFreekick(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::TheirFreekick::applicable()
{
	return gameState().setupRestart() && gameState().theirFreeKick();
}

bool Gameplay::Plays::TheirFreekick::run()
{
	return true;
}
