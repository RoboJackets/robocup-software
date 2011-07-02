#include "DemoYank.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoYank, "Demos")

Gameplay::Plays::DemoYank::DemoYank(GameplayModule *gameplay):
Play(gameplay),
_yank(gameplay)
{
	_dribblerSpeed = config()->createInt("DemoYank/Dribber Speed", 127);
	_enableBump = config()->createBool("DemoYank/Enable Bump", false);
}

bool Gameplay::Plays::DemoYank::run()
{
	Geometry2d::Point ballPos = ball().pos;

	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_yank.robot, available, ballPos);

	// if we have kicked, we want to reset
	if (_yank.done() &&  ball().valid &&
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		_yank.restart();
	}

	// set flags from parameters
	_yank.dribble_speed = *_dribblerSpeed;
	_yank.enable_bump = *_enableBump;

	_yank.run();

	return true;
}
