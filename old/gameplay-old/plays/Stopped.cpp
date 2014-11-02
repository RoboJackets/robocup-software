#include "Stopped.hpp"

#include <stdio.h>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::Stopped, "Restarts")

Gameplay::Plays::Stopped::Stopped(GameplayModule *gameplay):
	Play(gameplay),
	_idle(gameplay),
	_left(gameplay, Behaviors::Defender::Left),
	_right(gameplay, Behaviors::Defender::Right)
{
	_left.otherDefenders.insert(&_right);
	_right.otherDefenders.insert(&_left);
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
