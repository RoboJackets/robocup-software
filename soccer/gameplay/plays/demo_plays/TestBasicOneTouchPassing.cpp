#include <iostream>
#include <boost/foreach.hpp>
#include "TestBasicOneTouchPassing.hpp"

using namespace Geometry2d;
using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestBasicOneTouchPassing, "Tests")

Gameplay::Plays::TestBasicOneTouchPassing::TestBasicOneTouchPassing(GameplayModule *gameplay):
	Play(gameplay, 2),
	_passer(gameplay),
	_receiver(gameplay)
{
}

bool Gameplay::Plays::TestBasicOneTouchPassing::applicable()
{
	return true;
}

bool Gameplay::Plays::TestBasicOneTouchPassing::assign(set<Robot *> &available)
{
	cout << "available robots:  ";
	BOOST_FOREACH(Robot * r, available)
		cout << r->id() << " ";
	cout << endl;

	if (_robots.size() >= _minRobots)
		return false;

	_passer.assign(available);
	cout << "Passer ID: " << _passer.robot()->id() << endl;
	_receiver.assign(available);
	cout << "Reciever ID: " << _receiver.robot()->id() << endl;

	_passer.targetRobot = _receiver.robot();

	cout << "TestBasicOneTouchPassing::assign() successful!" << endl;
	return true;
}

bool Gameplay::Plays::TestBasicOneTouchPassing::run()
{
	bool verbose = false;
	if (verbose) cout << "PLAY: Running TestBasicOneTouchPassing" << endl;

	bool done = _passer.getState() == Behaviors::OneTouchKick::Done;
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

	_receiver.face = _passer.robot()->pos();
	_receiver.target = _receiver.robot()->pos();

	_passer.run();
	_receiver.run();

	// indicate which robot is receiving
	state()->drawCircle(_receiver.robot()->pos(), Constants::Robot::Radius*1.2, Qt::gray);

	if (verbose) cout << "PLAY: Finished Running TestBasicOneTouchPassing" << endl;

	return true;
}
