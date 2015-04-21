#include "DemoTouchKick.hpp"


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
	_kicker(gameplay),
	_passer(gameplay)
{
	_passTarget = Geometry2d::Point(1,4);
}

bool Gameplay::Plays::DemoTouchKick::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_kicker.robot, available, Geometry2d::Point());
	assignNearest(_passer.robot, available, Geometry2d::Point());

	Geometry2d::Point ballPos = ball().pos;

	_passer.setTarget(_kicker.robot->kickerBar());
	_passer.use_line_kick = true;

	// if we have kicked, we want to reset
	if (_kicker.done() &&  ball().valid && 
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		_kicker.restart();
		_passer.restart();
	}
	
	if (_passer.done() && ball().valid)
	{

	}

	if (_kicker.robot->pos.distTo(_passTarget) < 0.5)
	{
		_kicker.run();
	}
	else
	{
		_kicker.robot->move(_passTarget);
	}

	_kicker.use_chipper = *_use_chipper;
	_kicker.kick_power = *_kick_power;

	_passer.run();

	return true;
}
