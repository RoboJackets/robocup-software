#include "Play.hpp"

Gameplay::Play::Play(GameplayModule *gameplay):
	Behavior(gameplay)
{
	_gameplay->_plays.insert(this);
}

Gameplay::Play::~Play()
{
	_gameplay->_plays.erase(this);
}

bool Gameplay::Play::applicable()
{
	return true;
}

float Gameplay::Play::score()
{
	return 0;
}
