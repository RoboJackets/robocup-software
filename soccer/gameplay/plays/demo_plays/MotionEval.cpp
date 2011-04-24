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
	_points[0] = Point(-1, 1);
	_points[1] = Point(1, 1);
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
	robot->move(p);
	robot->face(p);
	
	return true;
}
