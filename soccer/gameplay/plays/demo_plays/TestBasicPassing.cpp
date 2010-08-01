/*
 * TestBasicPassing.cpp
 *
 *  Created on: Nov 8, 2009
 *      Author: alexgc
 */

#include "TestBasicPassing.hpp"

using namespace Geometry2d;
using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestBasicPassing, "Tests")

Gameplay::Plays::TestBasicPassing::TestBasicPassing(GameplayModule *gameplay):
	Play(gameplay, 2),
	_passer(gameplay),
	_receiver(gameplay)
{
}

bool Gameplay::Plays::TestBasicPassing::applicable()
{
	return true;
}

bool Gameplay::Plays::TestBasicPassing::assign(set<Robot *> &available)
{
	_passer.assign(available);
	_receiver.assign(available);

	_passer.setTarget(_receiver.robot());
	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::TestBasicPassing::run()
{
	_passer.setVScale(0.8, 0.3);
	bool done = _passer.getState() == Behaviors::Kick::Done;
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

	return true;
}
