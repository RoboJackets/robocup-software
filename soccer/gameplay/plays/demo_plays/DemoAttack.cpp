#include "DemoAttack.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoAttack, "Demos")

Gameplay::Plays::DemoAttack::DemoAttack(GameplayModule *gameplay):
Play(gameplay),
_kicker(gameplay)
{
	_useChip = config()->createBool("DemoAttack/Enable Chipping", false);
	_dribblerSpeed = config()->createInt("DemoAttack/Dribber Speed", 127);
	_kickPower = config()->createInt("DemoAttack/Kicker Power", 127);
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
