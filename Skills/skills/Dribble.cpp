#include "Dribble.hpp"

#include <stdio.h>

#include <Geometry/Point2d.hpp>

#include "../Robot.hpp"

using namespace Packet;
using namespace Geometry;

Dribble::Dribble() :
	Skill(Packet::SkillCmd::Dribble)
{
	
}

Packet::MotionCmd::Robot Dribble::run()
{
	MotionCmd::Robot mCmd;
	mCmd.valid = true;
	
	if (!robot()->status().ballPresent)
	{
		robot()->releaseBaton();
		
		_status = SkillStatus::Failed;
	}
	else
	{
		//set need baton
		robot()->requestBaton();
		
		//keep roller at slow speed to just hold onto ball
		mCmd.roller = 40;
		
		//position is final destination of the ball minus some N
		//ball should end up @ final destination
		mCmd.pos -= (_cmd.motion.pos - robot()->self().pos).norm() * .1;
		
		//path to destination
		Point2d path = _cmd.motion.pos - robot()->self().pos;
		
		//limit the path length, this limits the speed
		if (path.mag() > .2)
		{
			path = path.norm() * .2;
			mCmd.pos = robot()->self().pos + path;
		}
		
		//TODO maybe face the ball??
		mCmd.face = _cmd.motion.pos;
		
		//TODO this should be a settable threshold
		if (mCmd.pos.distTo(robot()->self().pos) < .02)
		{
			_status = SkillStatus::Done;
		}
		
		_status = SkillStatus::Running;
	}
	
	return mCmd;
}
