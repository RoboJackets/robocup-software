/*
 * SYTest.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: Matthew Barulic
 */

#include "SYTest.hpp"

namespace Gameplay {
namespace Plays {

REGISTER_PLAY_CATEGORY(SYTest, "Test");

SYTest::SYTest(GameplayModule *gameplay):
		Play(gameplay),
		_yanker(gameplay)
{
	_yanker.actionTarget = Geometry2d::Point(0,Field_Length);
}

bool SYTest::run()
{
	std::set<OurRobot *> available = _gameplay->playRobots();

	assignNearest(_yanker.robot, available, Geometry2d::Point());

	if(_yanker.isDone()) {
		_yanker.restart();
	}

	_yanker.run();

	return true;
}

} /* namespace Plays */
} /* namespace Gameplay */
