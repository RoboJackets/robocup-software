#include "Dribble.hpp"

#include <Geometry/Point2d.hpp>

using namespace Geometry;

Tactics::Factory_Type<Tactics::Dribble> dribble("dribble");

Tactics::Dribble::Dribble(Role *role) :
    Base(role),
    _pos1(this, "pos1"),
    _pos2(this, "pos2")
{
	_state = NextLoc;
}

void Tactics::Dribble::run()
{
	Packet::SkillCmd::Robot& skill = *robot()->skill();
	skill.valid = true;
	
	if (_state == NextLoc)
	{
		if (_nextLoc == _pos1.point())
		{
			_nextLoc = _pos2.point();
		}
		else
		{
			_nextLoc = _pos1.point();
		}
		
		_state = AcquireBall;
	}
	
	if (_state == AcquireBall)
	{
		//get the ball, facing the destination direction
		skill.skill = Packet::SkillCmd::GotoBall;
		skill.motion.face = _nextLoc;
		
		if (robot()->skill_status_code() == Packet::SkillStatus::Done)
		{
			//_state = Reorient;
			robot()->clear_status();
		}
	}
	
	if (_state == Reorient)
	{
		skill.skill = Packet::SkillCmd::Orient;
		skill.motion.face = _nextLoc;
		
		switch (robot()->skill_status_code())
		{
			case Packet::SkillStatus::Failed:
				_state = AcquireBall;
				robot()->clear_status();
				break;
			case Packet::SkillStatus::Done:
				_state = Travel;
				robot()->clear_status();
				break;
			default:
				break;
		}
	}
	
	if (_state == Travel)
	{	
		//reset point
		skill.skill = Packet::SkillCmd::Dribble;
		skill.motion.pos = _nextLoc;
		
		switch (robot()->skill_status_code())
		{
			case Packet::SkillStatus::Failed:
				_state = AcquireBall;
				robot()->clear_status();
				break;
			case Packet::SkillStatus::Done:
				_state = NextLoc;
				robot()->clear_status();
				break;
			default:
				break;
		}
	}
}
