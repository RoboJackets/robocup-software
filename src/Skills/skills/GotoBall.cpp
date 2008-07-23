#include "GotoBall.hpp"

#include <stdio.h>

#include <Geometry/Point2d.hpp>

#include "../Robot.hpp"

using namespace Packet;
using namespace Geometry;

GotoBall::GotoBall() :
	Skill(Packet::SkillCmd::GotoBall)
{
}

void GotoBall::start()
{
	_state = InitPos;
	_settleCount = 0;
}

Packet::MotionCmd::Robot GotoBall::run()
{
	Packet::MotionCmd::Robot mCmd;
	mCmd.valid = true;
	
	//settle the ball, finish skill
	if (robot()->status().ballPresent)
	{
		// turn roller on to hold ball
		mCmd.roller = 50;
		
		// reset state for ball loss
		_state = InitPos;
		
		//wait for ball to settle
		if (_settleCount++ > 20)
		{
			//release the baton
			robot()->releaseBaton();
			
			//skill is done
			_status = SkillStatus::Done;
		}
		
		//face the ball
		mCmd.face = robot()->vision().ball.pos;
		//hold position
		mCmd.pos = robot()->self().pos;
	}
	//get the ball
	else
	{
		_settleCount = 0;
		
		//stop the ball first
		if (_state == InitPos)
		{
			//don't touch the ball while getting into initial position
			mCmd.avoid = true;
			mCmd.avoidZone = Circle2d(robot()->vision().ball.pos, .1);
			
			//ball position and distance to go behind it
			const Point2d bPos = robot()->vision().ball.pos;
			const float dist = .2;
			
			//destination for robot defaults to vector from face to ball
			Point2d dest = _cmd.motion.face - bPos;
			
			//if ball is moving
			//we need to intercept it first
			//this means getting in its path
			if (robot()->vision().ball.vel.mag() > 0.5)
			{
				//path of ball
				dest = bPos + robot()->vision().ball.vel.norm() * dist;
			}
			else
			{
				dest = bPos + dest.norm() * - dist;
			}
			
			//face the ball. TODO think about maybe facing final location
			//this isn't always desired
			//mCmd.face =  robot()->vision().ball.pos;
			mCmd.face =  _cmd.motion.face;
			mCmd.pos = dest;
			
			mCmd.style = MotionCmd::Accurate;
			
			//threshold stop
			if (dest.distTo(robot()->self().pos) < .02f)
			{
				_state = Approach;
			}
		}
		//acquire tha ball
		else if (_state == Approach)
		{
			//set need baton
			robot()->requestBaton();
			
			//turn roller on for approach
			mCmd.roller = 50;
			
			//allowed near ball
			mCmd.avoid = false;
			
			mCmd.style = MotionCmd::Accurate;
			
			//destination is the ball
			const Point2d dest = robot()->vision().ball.pos;
			
			mCmd.face = dest;
			mCmd.pos = dest;
		}
	}
	
	return mCmd;
}

