#include "Kick.hpp"

#include <stdio.h>

#include "../Robot.hpp"

using namespace Packet;

Kick::Kick() :
	Skill(Packet::SkillCmd::Kick)
{
}

Packet::MotionCmd::Robot Kick::run()
{
	Packet::MotionCmd::Robot mCmd;
	mCmd.valid = true;
	
	if (robot()->status().ballPresent && robot()->status().charged)
	{
		//set need baton
		robot()->requestBaton();
		
		//kick with requested power
		mCmd.kick = _cmd.motion.kick;
		
		//keep roller off for some count..
		mCmd.roller = 20;
		
		mCmd.face = robot()->vision().ball.pos;
		mCmd.pos = robot()->self().pos;
        printf("kick pos %f, %f\n", mCmd.pos.x, mCmd.pos.y);
		
		_status = SkillStatus::Running;
	}
	//TODO failed state
	else if (!robot()->status().ballPresent || !robot()->status().charged)
	{
		//release the baton
		robot()->releaseBaton();
		
		_status = SkillStatus::Done;
	}
	
	return mCmd;
}
