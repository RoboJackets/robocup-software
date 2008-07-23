#include "Kick.hpp"

#include <Sizes.h>
#include <Geometry/Point2d.hpp>

using namespace Geometry;

Tactics::Factory_Type<Tactics::Kick> kick("kick");

Tactics::Kick::Kick(Role *role) :
    Base(role),
    _pos_param(this, "pos"),
    _strength(this, "strength", 255.0)
{
	_maxVel = 0;
}

float Tactics::Kick::score(Robot *robot)
{
    return robot->pos().distTo(vision_packet.ball.pos);
}

void Tactics::Kick::run()
{
	Packet::SkillCmd::Robot& skill = *robot()->skill();
	
	if (_state == InitPos)
	{
		switch (robot()->skill_status_code())
		{
			case Packet::SkillStatus::None:
			case Packet::SkillStatus::Running:
				break;
			case Packet::SkillStatus::Done:	
				_state = KickBall;
				break;
			case Packet::SkillStatus::Failed:
				_state = Done;
				break;
		}
	}
	else if (_state == KickBall)
	{
		switch (robot()->skill_status_code())
		{
			case Packet::SkillStatus::None:
			case Packet::SkillStatus::Running:
				break;
			case Packet::SkillStatus::Done:
			case Packet::SkillStatus::Failed:
				_state = Done;
				break;
		}	
	}
		
	robot()->clear_status();
	
	const float bMag = vision_packet.ball.vel.mag();
	if (bMag > _maxVel)
	{
		_maxVel = bMag;
	}
	
	if (_state == InitPos)
	{
		//printf("InitPos\n");
		skill.valid = true;
		skill.skill = Packet::SkillCmd::GotoBall;
		skill.motion.style = Packet::MotionCmd::Accurate;
		
		skill.motion.face = _pos_param.point();
	}
	else if (_state == KickBall)
	{
		//printf("Kick\n");
		skill.valid = true;
		
		//kick at varying speeds
		skill.motion.kick = (uint8_t)(_strength.value());
		
		skill.motion.face = _pos_param.point();
		skill.motion.pos = robot()->vision()->pos;
		
		skill.skill = Packet::SkillCmd::Kick;
		
		//TODO start recording max vel
	}
	else if (_state == Done)
	{
		//printf("Done\n");
		skill.valid = true;
		skill.skill = Packet::SkillCmd::None;
		
		skill.motion.face = Point2d(0, 0);
		skill.motion.pos = robot()->vision()->pos;
	}
}
