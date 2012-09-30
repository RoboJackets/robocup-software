#include "TestDefense.hpp"

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestDefense, "Test")

Gameplay::Plays::TestDefense::TestDefense(GameplayModule *gameplay):
	Play(gameplay),
	_fullbackLeft(gameplay, Behaviors::Fullback::Left),
	_fullbackRight(gameplay, Behaviors::Fullback::Right)
{
}

bool Gameplay::Plays::TestDefense::run()
{
	std::set<OurRobot *> available = _gameplay->playRobots();

	assignNearest(_fullbackLeft.robot, available, Geometry2d::Point());
	assignNearest(_fullbackRight.robot, available, Geometry2d::Point());

	_fullbackLeft.run();
	_fullbackRight.run();

	return true;
}
