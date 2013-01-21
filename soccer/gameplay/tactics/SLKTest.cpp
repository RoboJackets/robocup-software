#include "SLKTest.hpp"

REGISTER_PLAY_CATEGORY(Gameplay::Plays::SLKTest, "Test")

Gameplay::Plays::SLKTest::SLKTest(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay)
{
}

bool Gameplay::Plays::SLKTest::run()
{
	std::set<OurRobot *> available = _gameplay->playRobots();

	assignNearest(_kicker.robot, available, Geometry2d::Point());

	if(_kicker.done()) {
		_kicker.restart();
	}

	_kicker.run();

	return true;
}
