#include <iostream>
#include "Play.hpp"

Gameplay::Play::Play(GameplayModule *gameplay, size_t minRobots):
	Behavior(gameplay, minRobots)
{
}

bool Gameplay::Play::applicable()
{
	return true;
}

float Gameplay::Play::score()
{
	return 1.0;
}
