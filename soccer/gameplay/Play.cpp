#include <iostream>
#include "Play.hpp"

Gameplay::Play::Play(GameplayModule *gameplay):
	Behavior(gameplay)
{
}

bool Gameplay::Play::applicable()
{
	return true;
}

float Gameplay::Play::score()
{
	return 0;
}
