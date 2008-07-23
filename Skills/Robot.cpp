#include "Robot.hpp"

#include <stdio.h>

#include <Sizes.h>
#include <Geometry/Point2d.hpp>

#include "skills/Kick.hpp"
#include "skills/Dribble.hpp"
#include "skills/Orient.hpp"
#include "skills/Kick.hpp"
#include "skills/None.hpp"
#include "skills/GotoBall.hpp"
#include "skills/Steal.hpp"
#include "skills/Receive.hpp"
#include "skills/Mark.hpp"

using namespace Packet;
using namespace Geometry;

Robot::Robot(unsigned int id, Packet::VisionData& vd, Packet::RobotStatus::Robot& status) :
	_id(id), _vision(vd), _self(vd.self[id]), _status(status)
{
	//start off with default none skill
	_skill = 0;
	_needBaton = false;
}

SkillStatus::Status Robot::skillStatus() const 
{
	if (_skill)
	{
		return _skill->status();
	}
	
	return Packet::SkillStatus::None;
}

Packet::MotionCmd::Robot Robot::proc(Packet::SkillCmd::Robot skillCmd)
{	
	MotionCmd::Robot mCmd;
	mCmd.valid = false;
	
	if (skillCmd.valid)
	{
		mCmd.goalie = skillCmd.motion.goalie;
		
		//check skill cmd type
		//if different from the current type
		//delete the old one
		//make a new skill of right type
		if (!_skill || _skill->type() != skillCmd.skill)
		{
			//on change of skill, give up the baton
			releaseBaton();
			
			if (_skill)
			{
				//remove the old skill
				_skill->robot(0);
				delete _skill;
			}
			
			switch (skillCmd.skill)
			{
				case SkillCmd::Pass:
				case SkillCmd::Kick:
					_skill = new Kick();
					break;
				case SkillCmd::StealBall:
					_skill = new Steal();
				case SkillCmd::Mark:
					_skill = new Mark();
					break;
				case SkillCmd::GotoBall:
					_skill = new GotoBall();
					break;
				case SkillCmd::Dribble:
					_skill = new Dribble();
					break;
				case SkillCmd::Orient:
					_skill = new Orient();
					break;
                case SkillCmd::Receive:
                    _skill = new Receive();
                    break;
				default:
					_skill = new None();
			}
			
			_skill->robot(this);
			_skill->start();
		}
		
		// set the new command
		_skill->setCmd(skillCmd);
		
		// run the skill
		mCmd = _skill->run();
	}
	else
	{
		_skill = new None(); 
	}
	
	return mCmd;
}
