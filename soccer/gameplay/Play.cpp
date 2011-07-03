#include <iostream>
#include "Play.hpp"

#include <boost/foreach.hpp>

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

////////
// Assigners

bool assignNearest(OurRobot *&role, std::set<OurRobot*>& robots, Geometry2d::Point pt, bool needVisible)
{
	if (role && (!needVisible || role->visible))
	{
		// Already has a usable robot
		robots.erase(role);
		return true;
	}
	
	OurRobot *bestRobot = 0;
	float bestDistSq = -1;
	BOOST_FOREACH(OurRobot *robot, robots)
	{
		float dsq = (robot->pos - pt).magsq();
		if (bestDistSq < 0 || dsq < bestDistSq)
		{
			bestDistSq = dsq;
			bestRobot = robot;
		}
	}
	
	if (bestRobot)
	{
		role = bestRobot;
		robots.erase(role);

		// set default flags - only gets reset at assignment, rather than each frame
		role->avoidOpponents(true);
		role->avoidAllTeammates(true);
		role->avoidBall(Ball_Radius);
	}
	
	return role != 0;
}

bool assignNearest(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt,
		bool hasChipper, bool hasKicker, bool hasEncoders, bool needVisible)
{
	if (role && (!needVisible || role->visible))
	{
		// Already has a usable robot
		robots.erase(role);
		return true;
	}

	OurRobot *bestRobot = 0;
	float bestDistSq = -1;
	BOOST_FOREACH(OurRobot *robot, robots)
	{
		// manage requirements
		if ((!hasChipper || robot->hasChipper()) &&
				(!hasKicker  || robot->hasKicker()) &&
				(!hasEncoders || robot->hasEncoders()))
		{
			continue; // robot fails requirements
		}

		float dsq = (robot->pos - pt).magsq();
		if (bestDistSq < 0 || dsq < bestDistSq)
		{
			bestDistSq = dsq;
			bestRobot = robot;
		}
	}

	if (bestRobot)
	{
		role = bestRobot;
		robots.erase(role);

		// set default flags - only gets reset at assignment, rather than each frame
		role->avoidOpponents(true);
		role->avoidAllTeammates(true);
		role->avoidBall(Ball_Radius);
	}

	return role != 0;
}
