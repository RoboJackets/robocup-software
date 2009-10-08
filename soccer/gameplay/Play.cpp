#include "Play.hpp"
#include "Behavior.hpp"
#include "GameplayModule.hpp"

Gameplay::Play::Play(GameplayModule *gameplay)
{
	_gameplay = gameplay;
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

void Gameplay::Play::start()
{
}

void Gameplay::Play::stop()
{
}
