#include "SLKTest.hpp"
#include <gameplay/tactics/StableLineKick.hpp>
#include <gameplay/tactics/passing/PassReceiver.hpp>

REGISTER_PLAY_CATEGORY(Gameplay::Plays::SLKTest, "Test")

Gameplay::Plays::SLKTest::SLKTest(GameplayModule *gameplay):
	Play(gameplay),
	_passer(0),
	_receiver(0)
{
	_passer = new StableLineKick(gameplay);
	_receiver = new PassReceiver(gameplay);

	Geometry2d::Point receiveTarget(1.5,0.8);

	_passer->actionTarget = receiveTarget;
	_receiver->actionTarget = receiveTarget;

	_passer->partner = _receiver;
	_receiver->partner = _passer;
}

Gameplay::Plays::SLKTest::~SLKTest()
{
	if(_passer)
		delete _passer;
	if(_receiver)
		delete _receiver;
}

bool Gameplay::Plays::SLKTest::run()
{
	using namespace Geometry2d;

	if(!_passer || !_receiver) {
		return false;
	}

	std::set<OurRobot *> available = _gameplay->playRobots();

	if(!_passer->robot)
		assignNearest(_passer->robot, available, Point());
	if(!_receiver->robot)
		assignNearest(_receiver->robot, available, Point());

	if(!_passer->robot || !_receiver->robot) {
		return false;
	}

	if(_passer->isDone() && _receiver->isDone()) {
		_passer->restart();
		_receiver->restart();
	}

	_passer->run();
	_receiver->run();

	return true;
}
