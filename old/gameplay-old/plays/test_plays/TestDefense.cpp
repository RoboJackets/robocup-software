#include "TestDefense.hpp"

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestDefense, "Test")

Gameplay::Plays::TestDefense::TestDefense(GameplayModule *gameplay):
	Play(gameplay),
	_defenderLeft(gameplay, Behaviors::Defender::Left),
	_defenderRight(gameplay, Behaviors::Defender::Right)
{
}

bool Gameplay::Plays::TestDefense::run()
{
	std::set<OurRobot *> available = _gameplay->playRobots();

	assignNearest(_defenderLeft.robot, available, Geometry2d::Point());
	assignNearest(_defenderRight.robot, available, Geometry2d::Point());

	_defenderLeft.run();
	_defenderRight.run();

	return true;
}
