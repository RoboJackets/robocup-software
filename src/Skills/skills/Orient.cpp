#include "Orient.hpp"

#include <stdio.h>

#include <Geometry/Point2d.hpp>

#include "../Robot.hpp"

using namespace Packet;
using namespace Geometry;

Orient::Orient() :
	Skill(Packet::SkillCmd::Orient)
{
}

void Orient::start()
{
	_ball = robot()->vision().ball.pos;
}

Packet::MotionCmd::Robot Orient::run()
{
	MotionCmd::Robot mCmd;
	mCmd.valid = true;
	
	//keep roller at slow speed
	mCmd.roller = 60;
	
	mCmd.avoid = true;
	mCmd.avoidZone = Circle2d(robot()->vision().ball.pos, .005);
	
	//the distance behind ball should be calibrated
	Point2d dest = (_cmd.motion.face - _ball).norm() * -.07;
	mCmd.pos = _ball + dest;
	mCmd.face = _ball;
	
	//calculate facing error and check for done
	const float dist = robot()->self().pos.distTo(_cmd.motion.face);
	Point2d facing(cos(robot()->self().theta * M_PI/180.0), 
			sin(robot()->self().theta * M_PI/180.0));
	facing *= dist;
	facing += robot()->self().pos;
	
	const float faceErr = facing.distTo(_cmd.motion.face);
	printf("Face Err: %f go to %f, %f\n", faceErr, mCmd.pos.x, mCmd.pos.y);
	
	if (faceErr < .25f)
	{
		_status = SkillStatus::Done;
	}
	else
	{
		_status = SkillStatus::Running;
	}
	
	return mCmd;
}
