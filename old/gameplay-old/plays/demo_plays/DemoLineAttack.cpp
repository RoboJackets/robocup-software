#include "DemoLineAttack.hpp"


using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoLineAttack, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(DemoLineAttack)
	}
}

ConfigBool *Gameplay::Plays::DemoLineAttack::_use_chipper;
ConfigInt  *Gameplay::Plays::DemoLineAttack::_kick_power;

void Gameplay::Plays::DemoLineAttack::createConfiguration(Configuration* cfg)
{
	_use_chipper  = new ConfigBool(cfg, "DemoLineAttack/Enable Chipping", false);
	_kick_power = new ConfigInt(cfg, "DemoLineAttack/Kicker Power", 127);
}

Gameplay::Plays::DemoLineAttack::DemoLineAttack(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay)
{
}

bool Gameplay::Plays::DemoLineAttack::run()
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
