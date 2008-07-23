#include "None.hpp"

#include <stdio.h>

#include "../Robot.hpp"

None::None() :
	Skill(Packet::SkillCmd::None)
{
}

Packet::MotionCmd::Robot None::run()
{
	//pass motion commands straight through
	Packet::MotionCmd::Robot mCmd = _cmd.motion;
	mCmd.valid = true;
	
	return mCmd;
}
