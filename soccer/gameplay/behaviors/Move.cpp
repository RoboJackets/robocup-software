#include "Move.hpp"

Gameplay::Behaviors::Move::Move(GameplayModule *gameplay):
	Behavior(gameplay)
{
	backoff = 0;
}

float Gameplay::Behaviors::Move::score(Robot* r)
{
	return r->pos().distTo(target);
}

bool Gameplay::Behaviors::Move::run()
{
	if (!allVisible())
	{
		return false;
	}
	
	robot()->move(target - (target - robot()->pos()).normalized() * backoff);
	robot()->face(face);
	
	return false;
}
