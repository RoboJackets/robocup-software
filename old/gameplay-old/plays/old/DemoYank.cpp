#include "DemoYank.hpp"


using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoYank, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(DemoYank)
	}
}

ConfigInt *Gameplay::Plays::DemoYank::_dribblerSpeed;
ConfigBool *Gameplay::Plays::DemoYank::_enableBump;

void Gameplay::Plays::DemoYank::createConfiguration(Configuration *cfg)
{
	_dribblerSpeed = new ConfigInt(cfg, "DemoYank/Dribber Speed", 127);
	_enableBump  = new ConfigBool(cfg, "DemoYank/Enable Bump", false);
}

Gameplay::Plays::DemoYank::DemoYank(GameplayModule *gameplay):
Play(gameplay),
_yank(gameplay)
{
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
