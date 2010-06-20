#include "Forward.hpp"

using namespace std;
using namespace Geometry2d;

Gameplay::Behaviors::Forward::Forward(GameplayModule * gameplay):
	Behavior(gameplay, 1),
	_kick(gameplay)
{
	teammate = 0;
}

bool Gameplay::Behaviors::Forward::assign(set<Robot *> &available)
{
	_robots.clear();
	if (!takeBest(available))
	{
	    return false;
	}
	
	_kick.assignOne(robot());
	return _robots.size() >= _minRobots;
}

bool Gameplay::Behaviors::Forward::run()
{
	if (!allVisible() || !ball().valid)
	{
		return false;
	}
	
	//leave dribblers on
	robot()->dribble(20);

	// restart kicking to ensure that robots continue chasing the ball
	if (_kick.getState() == Kick::Done) {
		_kick.restart();
	}

	if (!teammate)
	{
		_kick.run();
		return false;
	}

	//Copy in important state data
	Point ball_pos = ball().pos;

	//get teammate position
	Point tm_pos = teammate->robot()->pos();
	Point tm_vel = teammate->robot()->vel();

	//check what other team is doing
	bool teammate_has_ball = !teammate->isIntercept();
	bool teammate_nearer_ball = robot()->pos().distTo(ball_pos) > tm_pos.distTo(ball_pos);
	if (teammate_has_ball)
	{
		//expect that the ball with go across the field
		bool ball_shot_on_left = ball_pos.x > 0;

		Point dest;
		if (ball_shot_on_left)
		{
			//go left
			dest = tm_pos + Point(-1.5,1);
		}
		else
		{
			//go right
			dest = tm_pos + Point(1.5,1);
		}
		robot()->move(dest);
		robot()->face(ball_pos);
	}
	else if (teammate_nearer_ball)
	{
		//back up along vector to ball
		Point dest = ball_pos + (robot()->pos() - ball_pos).normalized() * 0.5;
		robot()->move(dest);
		robot()->face(ball_pos);
	}
	else
	{
		//just run normal kick in auto mode
		_kick.run();
	}
	
	return false;
}

float Gameplay::Behaviors::Forward::score(Robot* robot)
{
	return Constants::Field::Length - robot->pos().y;
}

bool Gameplay::Behaviors::Forward::isIntercept()
{
	return _kick.isIntercept();
}
