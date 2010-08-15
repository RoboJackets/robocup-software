#include <iostream>
#include "Play.hpp"

std::list<Gameplay::PlayFactory *> *Gameplay::PlayFactory::_factories = 0;

Gameplay::PlayFactory::PlayFactory(QString c):
	category(c)
{
	enabled = false;
	lastScore = 0;
	
	if (!_factories)
	{
		_factories = new std::list<Gameplay::PlayFactory *>();
	}
	
	_factories->push_back(this);
}

const std::list<Gameplay::PlayFactory *>& Gameplay::PlayFactory::factories()
{
	if (!_factories)
	{
		_factories = new std::list<PlayFactory *>();
	}
	
	return *_factories;
}

////////

Gameplay::Play::Play(GameplayModule *gameplay):
	Behavior(gameplay)
{
}

float Gameplay::Play::score(GameplayModule *gameplay)
{
	return gameplay->state()->gameState.playing() ? 0 : INFINITY;
}
