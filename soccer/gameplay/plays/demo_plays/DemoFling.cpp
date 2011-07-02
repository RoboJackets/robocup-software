#include "DemoFling.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoFling, "Demos")

Gameplay::Plays::DemoFling::DemoFling(GameplayModule *gameplay):
Play(gameplay),
_fling(gameplay)
{
	_dribblerSpeed = config()->createInt("DemoFling/Dribber Speed", 127);
	_spinAngularSpeed = config()->createDouble("Fling/Spin Angular Speed", 2.0);
}

bool Gameplay::Plays::DemoFling::run()
{
	Geometry2d::Point ballPos = ball().pos;

	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_fling.robot, available, ballPos);

	// if we have kicked, we want to reset
	if (_fling.done() &&  ball().valid &&
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		_fling.restart();
	}

	// set flags from parameters
	_fling.dribble_speed = *_dribblerSpeed;
	_fling.spin_speed = *_spinAngularSpeed;

	_fling.run();

	return true;
}
