#include "Penalty.hpp"

using namespace std;

Gameplay::Behaviors::Penalty::Penalty(GameplayModule *gameplay):
	Behavior(gameplay),
	_kick(gameplay)
{
}

bool Gameplay::Behaviors::Penalty::assign(set<Robot *> &available)
{
	_robots.clear();
	if (!takeBest(available))
	{
	    return false;
	}
	_kick.assignOne(robot());

	return true;
}

bool Gameplay::Behaviors::Penalty::run()
{
	if (!allVisible() || !ball().valid)
	{
		return false;
	}
	
	Geometry2d::Point penaltyMark = Geometry2d::Point(0, Constants::Field::Length - Constants::Field::PenaltyDist);
	float backoff = 0.5f;
	
	switch (gameState().state)
	{
		case GameState::Setup:
			if (ball().pos.nearPoint(penaltyMark, 0.5))
			{
				robot()->move(ball().pos + (ball().pos - Geometry2d::Point(0, Constants::Field::Length)).normalized() * backoff);
			} else {
				robot()->move(penaltyMark - Geometry2d::Point(0, backoff));
			}
			robot()->face(ball().pos);
			break;

		case GameState::Ready:
			robot()->move(ball().pos + Geometry2d::Point(0, 0.1));
			robot()->face(ball().pos);
			robot()->kick(255);
			
			_kick.automatic = false;
			_kick.run();
			break;

		default:
			robot()->face(ball().pos);
			break;
	}
	
	return gameState().state != GameState::Playing;
}
