#include "MotionEval.hpp"

#include <stdio.h>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::MotionEval, "Demos")

Gameplay::Plays::MotionEval::MotionEval(GameplayModule *gameplay):
	Play(gameplay)
{
	_reached = false;
	
	_target = 0;
	_points.resize(2);
	_points[0] = Point(-1, 1.8);
	_points[1] = Point(1, 1.8);
// 	_points[0] = Point(-1.3, 0.7);
// 	_points[1] = Point(-1.3, 2.7);
}

float Gameplay::Plays::MotionEval::score(GameplayModule *gameplay)
{
	return 0;
}

bool Gameplay::Plays::MotionEval::run()
{
	OurRobot *robot = *_gameplay->playRobots().begin();
	if (!robot || !robot->visible)
	{
		return false;
	}
	
#if 1
	// Move to points in sequence
	const uint64_t now = _gameplay->state()->timestamp;
	
	Point p = _points[_target];
	
	if (robot->pos.nearPoint(p, 0.1) && !_reached)
	{
		_reached = true;
		_reachedTime = now;
	}
	
	if (_reached && (now - _reachedTime) >= 1000000)
	{
		_reached = false;
		_target = (_target + 1) % _points.size();
	}
	
	robot->addText(QString().sprintf("MotionEval %d %d", _target, _reached));
	state()->drawLine(_points[_target], _points[(_target - 1 + _points.size()) % _points.size()]);
 	robot->move(p);
	robot->face(_points[_target]);
// 	robot->face(ball().pos);
#endif
#if 0
	// Pivot test
	robot->dribble(31);
	
	if (robot->hasBall())
	{
		_reached = true;
	}
	
	if (_reached) //robot->hasBall())
	{
		robot->pivot(0.2 * M_PI, Ball_Radius + Robot_Radius);
// 		robot->directVelocityCommands(Point(1, 1).normalized() * 0.2, 0);
	}
#endif
#if 0
	// Angular rate test
	robot->directVelocityCommands(Point(), 2 * M_PI);
#endif
#if 0
	// Ball capture test
	if (state()->gameState.halt())
	{
		_target = 0;
	}
	
	static uint64_t startTime = 0;
	uint64_t now = Utils::timestamp();
	
	Geometry2d::Point toBall = (ball().pos - robot->pos).normalized();
	
	switch (_target)
	{
		case 0:
			// Waiting for ball
			robot->dribble(127);
			robot->worldVelocity(toBall * 0.5);
// 			int speed = min(127, 50 + (int)(now - startTime) * 127 * 2 / 1000000);
			if (robot->hasBall())
			{
				startTime = now;
				_target = 1;
			}
			break;
		
		case 1:
			robot->dribble(127);
			robot->worldVelocity(toBall * 0.5);
			if ((now - startTime) >= 2000000)
			{
				_target = 2;
			}
			break;
		
		case 2:
			robot->dribble(127);
			break;
	}
	
#endif
	
	return true;
}
