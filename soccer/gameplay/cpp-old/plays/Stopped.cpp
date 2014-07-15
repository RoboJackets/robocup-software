#include "Stopped.hpp"

#include <stdio.h>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::Stopped, "Restarts")

Gameplay::Plays::Stopped::Stopped(GameplayModule *gameplay):
	Play(gameplay),
	_idle(gameplay),
	_left(gameplay, Behaviors::Fullback::Left),
	_right(gameplay, Behaviors::Fullback::Right)
{
	_left.otherFullbacks.insert(&_right);
	_right.otherFullbacks.insert(&_left);
}

float Gameplay::Plays::Stopped::score(Gameplay::GameplayModule* gameplay)
{
	return gameplay->state()->gameState.stopped() ? 0 : INFINITY;
}

bool Gameplay::Plays::Stopped::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	assignNearest(_left.robot, available, Geometry2d::Point());
	assignNearest(_right.robot, available, Geometry2d::Point());
	_idle.robots = available;
	
	_idle.run();
	_left.run();
	_right.run();
	
	return true;
}
