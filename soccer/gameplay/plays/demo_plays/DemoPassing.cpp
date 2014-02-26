#include "DemoPassing.hpp"

using namespace Geometry2d;
using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoPassing, "Demos")

Gameplay::Plays::DemoPassing::DemoPassing(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_receiver(gameplay)
{
}

bool Gameplay::Plays::DemoPassing::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	if (!assignNearest(_passer.robot, available, Geometry2d::Point()) ||
			assignNearest(_receiver.robot, available, Geometry2d::Point()))
	{
		return false;
	}

	_passer.setTarget(_receiver.robot->kickerBar());

	bool done = _passer.done();
	if (done)
	{
		if (_doneTime.isNull())
		{
			_doneTime = QTime::currentTime();
		}
	} else {
		_doneTime = QTime();
	}

	if (!_gameplay->state()->gameState.playing() ||
		(!_doneTime.isNull() && _doneTime.msecsTo(QTime::currentTime()) >= 5000))
	{
		//cout << "Restarting kick behavior" << endl;
		_passer.restart();
	}

	_receiver.robot->face(_passer.robot->pos);
	_receiver.target = _receiver.robot->pos;

	_passer.run();
	_receiver.run();

	// indicate which robot is receiving
	state()->drawCircle(_receiver.robot->pos, Robot_Radius*1.2, Qt::gray);

	return true;
}
