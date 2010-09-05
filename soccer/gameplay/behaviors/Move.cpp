#include "Move.hpp"

Gameplay::Behaviors::Move::Move(GameplayModule *gameplay):
	SingleRobotBehavior(gameplay)
{
	backoff = 0;
}

bool Gameplay::Behaviors::Move::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	robot->move(target - (target - robot->pos).normalized() * backoff);
	robot->face(face);
	
	//FIXME - Threshold
	return false;
}
