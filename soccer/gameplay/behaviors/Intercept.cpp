#include "Intercept.hpp"
#include "../Role.hpp"

#include <Constants.hpp>
#include <LogFrame.hpp>
#include <vector>

#include <iostream>
#include <cmath>
using namespace std;
using namespace Packet;

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Intercept> behavior("intercept");

Gameplay::Behaviors::Intercept::Intercept(GameplayModule *gameplay, Role *role) :
	Behavior(gameplay, role), target_param(this, "target")
{

}

void Gameplay::Behaviors::Intercept::run()
{

	if (!ball().valid)
	{
		// No ball
		return;
	}

	const Geometry2d::Point pos = robot()->pos();

	// Get ball information
	const Geometry2d::Point ballPos = ball().pos;
	const Geometry2d::Point ballVel = ball().vel;

	// Always face the ball
	robot()->face(ballPos);

	const float farDist = .25;
	
	//if we already have the ball, skip approach states
	if (robot()->haveBall())
	{
		_state = Done;
	}
	else if (_state == ApproachBall)
	{
		if (!pos.nearPoint(ballPos, farDist))
		{
			_state = ApproachFar;
		}
	}
	
	//approach the ball at high speed facing the intended direction
	if (_state == ApproachFar)
	{
		Geometry2d::Point dest = ballPos;
		
		//if the ball is moving
		//we first need to try and intercept it
		if (ballVel.mag() > .1)
		{
			//look at where the ball will be 1 second from now
			//aka... the pos + vel
			dest += ballVel;
		}
		else
		{
			if (ballPos.nearPoint(pos, farDist))
			{
				dest += (ballPos - target_param.point()).normalized() * ballPos.distTo(pos);
			}
			else
			{
				dest += (ballPos - target_param.point()).normalized() * farDist;
			}
		}

		//TODO use the move behavior?
		robot()->move(dest);

		//create an obstacle to avoid the ball during this state
		ObstaclePtr ballObstacle(new CircleObstacle(ballPos, farDist
		        - Constants::Ball::Radius));

		const float dist = dest.distTo(pos);
		
		if (dist <= .05)
		{
			_state = ApproachBall;
		}
	}


	//approach the ball with intent to acquire it
	if (_state == ApproachBall)
	{
		//TODO change to just willHandle?
		//something more meaningful
		//we don't always want to kick
		robot()->willKick = true;

		robot()->move(ballPos);
		robot()->dribble(50);

		if (robot()->state()->haveBall)
		{
			_state = Done;

			//stop forward movement
			robot()->move(pos);
		}
	}
}

void Gameplay::Behaviors::Intercept::start()
{
	_state = ApproachFar;
}

bool Gameplay::Behaviors::Intercept::done()
{
	return _state == Done;
}

float Gameplay::Behaviors::Intercept::score(Robot * robot)
{
	Geometry2d::Point ball_pos = ball().pos;
	Geometry2d::Point robot_pos = robot->pos();
	return ball_pos.distTo(robot_pos);
}

