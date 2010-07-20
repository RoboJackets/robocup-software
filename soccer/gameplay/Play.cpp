#include <iostream>
#include "Play.hpp"

std::list<Gameplay::PlayFactoryBase *> *Gameplay::PlayFactoryBase::factories = 0;

Gameplay::PlayFactoryBase::PlayFactoryBase(QString c):
	category(c)
{
	if (!factories)
	{
		factories = new std::list<Gameplay::PlayFactoryBase *>();
	}
	
	factories->push_back(this);
}

////////

Gameplay::Play::Play(GameplayModule *gameplay, size_t minRobots):
	Behavior(gameplay, minRobots)
{
	enabled = false;
}

bool Gameplay::Play::applicable()
{
	return true;
}

float Gameplay::Play::score()
{
	return 1.0;
}
