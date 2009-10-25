#include "Play.hpp"

Gameplay::Play::Play(GameplayModule *gameplay):
	Behavior(gameplay)
{
	_gameplay->_AvailablePlays.insert(make_pair(this->name(), this));
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
