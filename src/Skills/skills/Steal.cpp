#include "Steal.hpp"

#include <stdio.h>

#include <Geometry/Point2d.hpp>

#include "../Robot.hpp"

using namespace Packet;
using namespace Geometry;

Steal::Steal() :
	Skill(Packet::SkillCmd::StealBall)
{
	_state = InitPos;
}

void Steal::start()
{
	_state = InitPos;
	_spinCount = 0;
}

Packet::MotionCmd::Robot Steal::run()
{
	_gotoBall.robot(this->robot());
	
	Packet::MotionCmd::Robot mCmd;
	mCmd.valid = true;
	
	_status = SkillStatus::Running;
	
	if (_state == InitPos)
	{
		//set need baton
		robot()->requestBaton();
		
		//setup the command
		//face the opponent
		_cmd.motion.face = _cmd.motion.pos;
		
		//run the gotoball comamnd first
		_gotoBall.setCmd(_cmd);
		
		mCmd = _gotoBall.run();
		
		if (_gotoBall.status() == SkillStatus::Done)
		{
			_state = Spin;
		}
	}
	else if (_state == Spin)
	{
		//give up the baton
		robot()->releaseBaton();
		
		mCmd.valid = true;
		mCmd.spin = true;
		
		if (_spinCount++ > 30)
		{
			_state = Done;
		}
	}
	else if (_state == Done)
	{
		_status = SkillStatus::Done;
		
		mCmd.valid = false;
		mCmd.spin = false;
	}
	
	return mCmd;
}

