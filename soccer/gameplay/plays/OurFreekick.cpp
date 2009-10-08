#include "OurFreekick.hpp"

Gameplay::Plays::OurFreekick::OurFreekick(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::OurFreekick::applicable()
{
	return gameState().setupRestart() && gameState().ourFreeKick();
}

bool Gameplay::Plays::OurFreekick::run()
{
	return true;
}
