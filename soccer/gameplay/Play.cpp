#include <iostream>
#include "Play.hpp"

Gameplay::Play::Play(GameplayModule *gameplay):
	Behavior(gameplay)
{
}

Gameplay::Play::~Play()
{
	_gameplay->_AvailablePlays.erase(this->name());
}

bool Gameplay::Play::applicable()
{
	return true;
}

float Gameplay::Play::score()
{
	return 0;
}
