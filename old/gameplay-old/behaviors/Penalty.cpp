#include "Penalty.hpp"

using namespace std;

Gameplay::Behaviors::Penalty::Penalty(GameplayModule *gameplay):
	SingleRobotBehavior(gameplay),
	_kick(gameplay)
{
}

bool Gameplay::Behaviors::Penalty::run()
{
	if (!robot || !robot->visible || !ball().valid)
	{
		return false;
	}
	
	_kick.robot = robot;
	
	Geometry2d::Point penaltyMark = Geometry2d::Point(0, Field_Length - Field_PenaltyDist);
	float backoff = 0.5f;
	
	switch (gameState().state)
	{
		case GameState::Setup:
			if (ball().pos.nearPoint(penaltyMark, 0.5))
			{
				robot->move(ball().pos + (ball().pos - Geometry2d::Point(0, Field_Length)).normalized() * backoff);
			} else {
				robot->move(penaltyMark - Geometry2d::Point(0, backoff));
			}
			robot->face(ball().pos);
			break;

		case GameState::Ready:
			robot->move(ball().pos + Geometry2d::Point(0, 0.1));
			robot->face(ball().pos);
			robot->kick(255);
			
// 			_kick.automatic = false;
			_kick.run();
			break;

		default:
			robot->face(ball().pos);
			break;
	}
	
	return gameState().state != GameState::Playing;
}
