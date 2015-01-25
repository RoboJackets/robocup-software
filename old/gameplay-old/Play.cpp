#include <iostream>
#include "Play.hpp"
#include <Robot.hpp>


#include <RobotConfig.hpp>

using namespace std;

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

bool assignNearest(OurRobot *&role, std::set<OurRobot*>& robots, Geometry2d::Point pt)
{
	if (role && role->visible)
	{
		// Already has a usable robot
		robots.erase(role);
		return true;
	}
	
	OurRobot *bestRobot = 0;
	float bestDistSq = -1;
	for (OurRobot *robot :  robots)
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
		role->avoidBallRadius(Ball_Radius);
	}
	
	return role != 0;
}

bool assignNearestFull(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt)
{
	return assignNearest(role, robots, pt, true, true, true, true);
}

bool assignNearestKicker(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt)
{
	return assignNearest(role, robots, pt, true, false, true, true);
}

bool assignNearestChipper(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt)
{
	return assignNearest(role, robots, pt, false, true, true, true);
}

bool assignNearestYank(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt)
{
	return assignNearest(role, robots, pt, false, false, true, true);
}

static bool meetsRequirements(OurRobot * robot,	bool hasKicker, bool hasChipper, bool hasDribbler, bool hasBallSense)
{
	return !((hasKicker    && !robot->kicker_available()) ||
			(hasChipper   && !robot->chipper_available()) ||
			(hasDribbler  && !robot->dribbler_available()) ||
			(hasBallSense && (!*robot->status->ball_sense_enabled || !robot->ballSenseWorks())));
}

// General "AssignNearest" with a full set of constraints - true requires that it be met
bool assignNearest(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt,
		bool hasKicker, bool hasChipper, bool hasDribbler, bool hasBallSense)
{
		if (role && role->visible && meetsRequirements(role, hasKicker, hasChipper, hasDribbler, hasBallSense))
		{
			// Already has a usable robot
			robots.erase(role);
			return true;
		}

		OurRobot *bestRobot = 0;
		float bestDistSq = -1;
		for (OurRobot *robot :  robots)
		{
			// manage requirements
			if (meetsRequirements(robot, hasKicker, hasChipper, hasDribbler, hasBallSense))
			{
				float dsq = (robot->pos - pt).magsq();
				if (bestDistSq < 0 || dsq < bestDistSq)
				{
					bestDistSq = dsq;
					bestRobot = robot;
				}
			}
		}

		if (bestRobot)
		{
			role = bestRobot;
			robots.erase(role);

			// set default flags - only gets reset at assignment, rather than each frame
			role->avoidOpponents(true);
			role->avoidAllTeammates(true);
			role->avoidBallRadius(Ball_Radius);
		}

		return role != 0 && meetsRequirements(role, hasKicker, hasChipper, hasDribbler, hasBallSense);
}
