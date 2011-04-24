#include "DemoBasicPivotAttack.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoBasicPivotAttack, "Demos")

Gameplay::Plays::DemoBasicPivotAttack::DemoBasicPivotAttack(GameplayModule *gameplay):
Play(gameplay),
_kicker(gameplay)
{
}

bool Gameplay::Plays::DemoBasicPivotAttack::run()
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

	_kicker.run();

	return true;
}
