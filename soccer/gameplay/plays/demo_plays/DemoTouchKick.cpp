#include "DemoTouchKick.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoTouchKick, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(DemoTouchKick)
	}
}

ConfigBool *Gameplay::Plays::DemoTouchKick::_use_chipper;
ConfigInt  *Gameplay::Plays::DemoTouchKick::_kick_power;

void Gameplay::Plays::DemoTouchKick::createConfiguration(Configuration* cfg)
{
	_use_chipper  = new ConfigBool(cfg, "TouchTest/Enable Chipping", false);
	_kick_power = new ConfigInt(cfg, "TouchTest/Kicker Power", 127);
}

Gameplay::Plays::DemoTouchKick::DemoTouchKick(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay)
{
}

bool Gameplay::Plays::DemoTouchKick::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_kicker.robot, available, Geometry2d::Point());

	Geometry2d::Point ballPos = ball().pos;

	// if we have kicked, we want to reset
	if (_kicker.done() &&  ball().valid && 
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		_kicker.restart();
	}
	
	_kicker.use_chipper = *_use_chipper;
	_kicker.kick_power = *_kick_power;

	_kicker.run();

	return true;
}
