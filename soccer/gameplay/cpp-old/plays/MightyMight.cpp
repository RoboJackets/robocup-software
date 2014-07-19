#include "MightyMight.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::MightyMight, "Playing")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(MightyMight)
	}
}


ConfigBool *Gameplay::Plays::MightyMight::_useLineKick;
ConfigBool *Gameplay::Plays::MightyMight::_defenseFirst;

void Gameplay::Plays::MightyMight::createConfiguration(Configuration *cfg)
{
	_useLineKick = new ConfigBool(cfg, "MightyMight/Use Line Kick", true);
	_defenseFirst = new ConfigBool(cfg, "MightyMight/Defense First", true);
}

Gameplay::Plays::MightyMight::MightyMight(GameplayModule *gameplay):
	Play(gameplay),
	_leftDefender(gameplay, Behaviors::Defender::Left),
	_rightDefender(gameplay, Behaviors::Defender::Right),
	_centerDefender(gameplay, Behaviors::Defender::Center),
	_kicker1(gameplay),
	_kicker2(gameplay)
{
	_leftDefender.otherDefenders.insert(&_rightDefender);
	_leftDefender.otherDefenders.insert(&_centerDefender);
	_rightDefender.otherDefenders.insert(&_leftDefender);
	_rightDefender.otherDefenders.insert(&_centerDefender);
	_centerDefender.otherDefenders.insert(&_rightDefender);
	_centerDefender.otherDefenders.insert(&_leftDefender);
}

float Gameplay::Plays::MightyMight::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::MightyMight::run()
{
	// handle assignments
	set<OurRobot *> available = _gameplay->playRobots();

	if(*_defenseFirst) {
	// defense first - closest to goal
		assignNearest(_leftDefender.robot, available, Geometry2d::Point());
		assignNearest(_rightDefender.robot, available, Geometry2d::Point());

	// choose offense, we want both robots to attack
		assignNearest(_kicker1.robot, available, ball().pos);
		assignNearest(_kicker2.robot, available, ball().pos);
	} else {
	// choose offense, we want both robots to attack
		assignNearest(_kicker1.robot, available, ball().pos);
		assignNearest(_kicker2.robot, available, ball().pos);

	// defense first - closest to goal
		assignNearest(_leftDefender.robot, available, Geometry2d::Point());
		assignNearest(_rightDefender.robot, available, Geometry2d::Point());
	}

	// additional defense - if it exists
	assignNearest(_centerDefender.robot, available, Geometry2d::Point());

	// manually reset any kickers so they keep kicking
	if (_kicker1.done())
		_kicker1.restart();
	if (_kicker2.done())
		_kicker2.restart();

	// add obstacles - always bias to kicker1
	if (_kicker1.robot && _kicker2.robot) {
		unsigned k1 = _kicker1.robot->shell(), k2 = _kicker2.robot->shell();
		_kicker1.robot->avoidTeammateRadius(k2, 0.8 * Robot_Radius);
		_kicker2.robot->avoidTeammateRadius(k1, 0.5);
	}



	_kicker1.use_line_kick = *_useLineKick;
	_kicker2.use_line_kick = *_useLineKick;


	// execute kickers dumbly
	if (_kicker1.robot) _kicker1.run();
	if (_kicker2.robot) _kicker2.run();

	// run standard defender behavior
	if (_leftDefender.robot) _leftDefender.run();
	if (_rightDefender.robot) _rightDefender.run();
	if (_centerDefender.robot) _centerDefender.run();
	
	return true;
}
