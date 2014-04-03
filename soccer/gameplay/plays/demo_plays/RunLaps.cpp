
#include "RunLaps.hpp"

using namespace std;
using namespace Geometry2d;


REGISTER_PLAY_CATEGORY(Gameplay::Plays::RunLaps, "Demos")

Gameplay::Plays::RunLaps::RunLaps(GameplayModule *gameplay):
	Play(gameplay)
{
	_targetingLeft = true;
	_stabilizeTimeout.setIntervalInSeconds(3);
}

float Gameplay::Plays::RunLaps::score(GameplayModule *gameplay)
{
	return 0;
}

bool Gameplay::Plays::RunLaps::run()
{
	set<OurRobot *> available = gameplay()->playRobots();

	//	target point
	float fudgeFactor = .15;
	float maxX = Field_Width / 2.0 - Robot_Radius - fudgeFactor;
	float x = _targetingLeft ? -maxX : maxX;
	float y = 0.8;
	Point target(x, y);


	OurRobot *robot = nullptr;
	assignNearest(robot, available, target);

	if (!robot) return false;

	robot->move(target);

	//	swap directions if we made it there
	if (!_arrived && (robot->pos - target).mag() < 0.02) {
		_arrived = true;
		_stabilizeTimeout.reset();
	}

	if (_arrived && _stabilizeTimeout.isTimedOut()) {
		_targetingLeft = !_targetingLeft;
		_arrived = false;
	}

	return true;
}
