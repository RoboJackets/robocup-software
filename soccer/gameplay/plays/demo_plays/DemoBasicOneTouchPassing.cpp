#include <iostream>
#include <boost/foreach.hpp>
#include "DemoBasicOneTouchPassing.hpp"

using namespace Geometry2d;
using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoBasicOneTouchPassing, "Demos")

Gameplay::Plays::DemoBasicOneTouchPassing::DemoBasicOneTouchPassing(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_receiver(gameplay)
{
	set<Robot *> available = gameplay->robots();
	cout << "available robots:  ";
	BOOST_FOREACH(Robot * r, available)
		cout << r->id() << " ";
	cout << endl;

	_passer.assign(available);
	cout << "Passer ID: " << _passer.robot()->id() << endl;
	_receiver.assign(available);
	cout << "Reciever ID: " << _receiver.robot()->id() << endl;

	_passer.targetRobot = _receiver.robot();

	cout << "DemoBasicOneTouchPassing::assign() successful!" << endl;
}

bool Gameplay::Plays::DemoBasicOneTouchPassing::run()
{
	bool verbose = false;
	if (verbose) cout << "PLAY: Running DemoBasicOneTouchPassing" << endl;

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

	if (verbose) cout << "PLAY: Finished Running DemoBasicOneTouchPassing" << endl;

	return true;
}
