#include "Behavior.hpp"
#include "GameplayModule.hpp"

#include <stdexcept>
#include <boost/foreach.hpp>

using namespace Geometry2d;
using namespace std;

Gameplay::Behavior::Behavior(GameplayModule *gameplay)
{
	_gameplay = gameplay;
	_robot = 0;
}

Gameplay::Behavior::~Behavior()
{
}

float Gameplay::Behavior::score(Robot *robot)
{
	return 0;
}

Gameplay::Robot *Gameplay::Behavior::selectRobot()
{
	float bestScore = 0;
	Robot *bestRobot = 0;
	BOOST_FOREACH(Robot *r, _gameplay->self)
	{
		if (r->visible())//FIXME && !r->behavior())
		{
			float s = score(r);
			if (!bestRobot || s < bestScore)
			{
				bestScore = s;
				bestRobot = r;
			}
		}
	}

	return bestRobot;
}

void Gameplay::Behavior::robot(Robot *robot)
{
	// If we were running, stop.
	if (_robot)
	{
		stop();
	}

	_robot = robot;

	// If we are now running, start.
	if (_robot)
	{
		start();
	}
}

void Gameplay::Behavior::start()
{
}

void Gameplay::Behavior::stop()
{
}

void Gameplay::Behavior::run()
{
}

bool Gameplay::Behavior::done()
{
	return true;
}
