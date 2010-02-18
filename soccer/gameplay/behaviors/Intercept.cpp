#include "Intercept.hpp"

#include <iostream>
#include <Constants.hpp>

#include <cmath>

using namespace std;

Gameplay::Behaviors::Intercept::Intercept(GameplayModule *gameplay, float dist) :
	Behavior(gameplay), _farDist(dist)
{
}

void Gameplay::Behaviors::Intercept::assign(set<Robot *> &available)
{
	takeBest(available);
	_state = ApproachFar;
}

float Gameplay::Behaviors::Intercept::score(Robot * robot)
{
	Geometry2d::Point ball_pos = ball().pos;
	Geometry2d::Point robot_pos = robot->pos();

	return ball_pos.distTo(robot_pos);
}

bool Gameplay::Behaviors::Intercept::run()
{
	if (!allVisible() || !ball().valid)
	{
		// No ball
		return false;
	}

	const Geometry2d::Point pos = robot()->pos();

	// Get ball information
	const Geometry2d::Point ballPos = ball().pos;
	const Geometry2d::Point ballVel = ball().vel;

	// determine where to put the debug text
	const Geometry2d::Point textOffset(Constants::Robot::Radius * -1.3, 0);

	// Always face the ball
	robot()->face(ballPos);

	// default to full speed unless we are close to ball
	robot()->setVScale(1.0f);

	//if we already have the ball, skip approach states
	if (robot()->haveBall())
	{
		_state = Done;
	}
	else if (_state == ApproachBall)
	{
		if (!pos.nearPoint(ballPos, _farDist))
		{
			_state = ApproachFar;
		}
	}

	//approach the ball at high speed facing the intended direction
	if (_state == ApproachFar)
	{
		drawText("ApproachFar", pos + textOffset, 0, 0, 0);
		Geometry2d::Point dest = ballPos;


		//if the ball is moving
		//we first need to try and intercept it
		bool skipBallVelocityCalc = true;
// This is causing problems when the ball even has a low velocity
// Disabled for now.
		if (!skipBallVelocityCalc && ballVel.mag() > .1)
		{
			// Project the destination ahead far enough to account for movement
			const float average_speed  = 2.0; // for the robot
			float travelTime = pos.distTo(ballPos)/average_speed;
			dest += ballVel*travelTime;

			//look at where the ball will be 1 second from now
			// changed to 0.45 seconds
			//aka... the pos + vel
			//dest += ballVel*0.2;
		}
		else
		{
			if (ballPos.nearPoint(pos, _farDist))
			{
				dest += (ballPos - target).normalized() * ballPos.distTo(pos);
			}
			else
			{
				dest += (ballPos - target).normalized() * _farDist;
			}
		}

		//TODO use the move behavior?
		robot()->move(dest);

		//create an obstacle to avoid the ball during this state
		ObstaclePtr ballObstacle(new CircleObstacle(ballPos, _farDist - Constants::Ball::Radius));

		const float dist = dest.distTo(pos);

		if (dist <= _farDist)
		{
			_state = ApproachBall;
		}
	}

	//approach the ball with intent to acquire it
	if (_state == ApproachBall)
	{
		drawText("ApproachBall", pos + textOffset, 0, 0, 0);
		//TODO change to just willHandle?
		//something more meaningful
		//we don't always want to kick
		robot()->willKick = true;

		// experimental 02/11/10
		// kickerGoalPos is the target, instead of the ballpos
		// hopefully, this will reduce ramming the ball.
		Geometry2d::Point kickerGoalPos(ballPos-target);
		kickerGoalPos = kickerGoalPos.normalized() * (Constants::Robot::Radius);
		kickerGoalPos += ballPos;

		robot()->move(kickerGoalPos);
		robot()->dribble(50);
		robot()->setVScale(0.8); // dampen velocity when near the ball

		if (robot()->haveBall())
		{
			_state = Done;

			//stop forward movement
			robot()->move(pos);
		}
	}

	return _state != Done;
}
