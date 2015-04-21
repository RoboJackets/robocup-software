#include "DemoPivotAttack.hpp"


using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoPivotAttack, "Demos")

Gameplay::Plays::DemoPivotAttack::DemoPivotAttack(GameplayModule *gameplay):
Play(gameplay),
_kicker(gameplay)
{
	_kicker.target = Segment(Point(-Field_GoalWidth/2.0,Field_Length-0.1), Point(Field_GoalWidth/2.0,Field_Length-0.1));
}

bool Gameplay::Plays::DemoPivotAttack::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_kicker.robot, available, Geometry2d::Point());

	Geometry2d::Point ballPos = ball().pos;

	_kicker.use_chipper = true;

	// if we have kicked, we want to reset
	if (_kicker.done() &&  ball().valid && 
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		_kicker.restart();
	}

	_kicker.run();

	return true;
}
