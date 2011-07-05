#include "DemoAttack.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoAttack, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(DemoAttack)
	}
}

ConfigBool *Gameplay::Plays::DemoAttack::_useChip;
ConfigInt  *Gameplay::Plays::DemoAttack::_dribblerSpeed;
ConfigInt  *Gameplay::Plays::DemoAttack::_kickPower;

void Gameplay::Plays::DemoAttack::createConfiguration(Configuration* cfg)
{
	_useChip  = new ConfigBool(cfg, "DemoAttack/Enable Chipping", false);
	_dribblerSpeed = new ConfigInt(cfg, "DemoAttack/Dribber Speed", 127);
	_kickPower = new ConfigInt(cfg, "DemoAttack/Kicker Power", 127);
}

Gameplay::Plays::DemoAttack::DemoAttack(GameplayModule *gameplay):
Play(gameplay),
_kicker(gameplay)
{
}

bool Gameplay::Plays::DemoAttack::run()
{
	Geometry2d::Point ballPos = ball().pos;

	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_kicker.robot, available, ballPos);

	// if we have kicked, we want to reset
	if (_kicker.done() &&  ball().valid && 
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		_kicker.restart();
	}

	// set flags from parameters
	_kicker.use_chipper = *_useChip;
	_kicker.dribbler_speed = *_dribblerSpeed;
	_kicker.kick_power = *_kickPower;

	_kicker.run();

	return true;
}
