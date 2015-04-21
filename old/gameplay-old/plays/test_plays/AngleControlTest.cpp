
#include "AngleControlTest.hpp"

using namespace std;
using namespace Geometry2d;
using namespace Gameplay;


REGISTER_PLAY_CATEGORY(Gameplay::Plays::AngleControlTest, "Test");

Gameplay::Plays::AngleControlTest::AngleControlTest(GameplayModule *gameplay) : Play(gameplay) {
	Point pt = Point(0, 1);
	_targets.push_back(pt + Point(1, 0));
	_targets.push_back(pt - Point(1, 0));
	_targets.push_back(pt + Point(0, 1));
	_targetIndex = 0;
}

float Gameplay::Plays::AngleControlTest::score(GameplayModule *gameplay) {
	return 0;
}

bool Gameplay::Plays::AngleControlTest::run() {
	Point pt = Point(0, 1);

	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_robot, available, pt);

	//	abort if no robot
	if (!_robot) return false;

	_robot->move(pt);

	//	if we timeout (keep in mind times are in microsec), we face the next target
	if (timestamp() - _targetStartTime > 5000000) {
		_targetIndex = (_targetIndex + 1) % _targets.size();
		_targetStartTime = timestamp();
	}


	// _robot->addText(QString("targetIndex: %1").arg(_targetIndex));


	Point target = _targets[_targetIndex];

	_robot->face(target);

	state()->drawCircle(target, 0.05, Qt::blue);

	return true;
}
