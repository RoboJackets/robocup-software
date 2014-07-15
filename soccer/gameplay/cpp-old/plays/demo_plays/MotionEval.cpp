#include "MotionEval.hpp"

#include <stdio.h>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::MotionEval, "Demos")

static bool shellLessThan(Robot *r1, Robot *r2)
{
	return r1->shell() < r2->shell();
}

FILE *fp;

static const float Goal_X = 0.70;
static const float Goal_Y = 0.70;

Gameplay::Plays::MotionEval::MotionEval(GameplayModule *gameplay):
	Play(gameplay)
{
	fp = fopen("test.txt", "wb");
	_lastAngle = 0;
	_lastTime = timestamp();
	
	_reached = false;
	
	_target = 0;
	_points.resize(2);
// 	_points[0] = Point(-0.2, 1.0);
// 	_points[1] = Point(0.2, 1.0);
	_points[0] = Point(-1, 1);
	_points[1] = Point(-1, 6);
// 	_points[0] = Point(Goal_X, Field_Length - Goal_Y);
// 	_points[1] = Point(-Goal_X, Field_Length - Goal_Y);
// 	_points[2] = Point(-Goal_X, Goal_Y);
// 	_points[3] = Point(Goal_X, Goal_Y);
}

float Gameplay::Plays::MotionEval::score(GameplayModule *gameplay)
{
	return 0;
}

bool Gameplay::Plays::MotionEval::run()
{
	if (_gameplay->playRobots().empty())
	{
		return false;
	}
	
#if 0
	OurRobot *robot = *_gameplay->playRobots().begin();
	if (!robot || !robot->visible)
	{
		return false;
	}
#endif

#if 1
	const set<OurRobot *> &playRobots = _gameplay->playRobots();
	if (playRobots.empty())
	{
		// No robots
		return false;
	}
	
	float backoff = 0.1;
	
	// Make a list of robots, sorted by shell
	vector<OurRobot *> robots(playRobots.size());
	copy(playRobots.begin(), playRobots.end(), robots.begin());
	sort(robots.begin(), robots.end(), shellLessThan);
	
	// Put the manual robot in front
	int manualID = _gameplay->manualID();
	for (unsigned int i = 0; i < robots.size(); ++i)
	{
		if ((int)robots[i]->shell() == manualID)
		{
			swap(robots[i], robots[0]);
			break;
		}
	}
	
	// Move to points in sequence
	const uint64_t now = _gameplay->state()->timestamp;
	
	Point p = _points[_target];
	
	if (robots[0]->pos.nearPoint(p, 0.1) && !_reached)
	{
		_reached = true;
		_reachedTime = now;
	}
	
	if (_reached && (now - _reachedTime) >= 250000)
	{
		_reached = false;
		_target = (_target + 1) % _points.size();
	}
	
// 	robot->addText(QString().sprintf("MotionEval %d %d", _target, _reached));
	state()->drawLine(_points[_target], _points[(_target - 1 + _points.size()) % _points.size()]);
 	robots[0]->move(p);
	robots[0]->face(_points[_target]);
// 	robot->face(ball().pos);
	
	// robot[0] is leader
	// robot[i] follows robot[i-1].
	for (unsigned int i = 1; i < robots.size(); ++i)
	{
		Geometry2d::Point leader = robots[i - 1]->pos;
		Geometry2d::Point cur = robots[i]->pos;
		// Stay a small distance behind the leader
		Geometry2d::Point dest = leader - (leader - cur).normalized() * (Robot_Diameter + backoff);
		robots[i]->move(dest);
		robots[i]->face(leader);
	}
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
	uint64_t now = timestamp();
	
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
