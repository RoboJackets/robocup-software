#include "Handler.hpp"
#include "../Role.hpp"

#include <list>
#include <Sizes.h>
#include <Geometry/Point2d.hpp>

#include "Forward.hpp"
#include "../Predicates.hpp"

using namespace std;
using namespace Geometry;
using namespace Packet;

Tactics::Factory_Type<Tactics::Handler> handler("handler");

Tactics::Handler::Handler(Role *role) :
    Base(role)
{
	_state = GetBall;
}

float Tactics::Handler::score(Robot* r)
{
	return (vision_packet.ball.pos - r->pos()).mag();
}

void Tactics::Handler::run()
{
	//the job of the handler is the be the one in control of the ball
	//therefore, only one handler tactic should be active at any given time

	//if there is more than one active, the handlers need to decide who will
	//actually control the ball

	//the handler tactic should only be active on a robot while it is in control
	//of the ball

	//assume we don't have the ball, until a skill tells us we do...

	//if we don't have the ball
	//set the skill and get it

	//if we have the ball...look for open pass?? ... move ball upfield
	
	Packet::SkillCmd::Robot& skill = *robot()->skill();
	skill.skill = SkillCmd::None;
	skill.valid = true;
	skill.motion.style = MotionCmd::Fast;
	
	if (penalty && our_action)
	{
		skill.motion.face = Point2d(0, 6.1);
		
		if (setup)
		{
			//go near ball, do not touch
			skill.motion.pos = Point2d(0, 5.5);
		}
		else if (running)
		{
			skill.skill = SkillCmd::GotoBall;
			skill.motion.style = MotionCmd::Accurate;
			
			//TODO turn on onetouch
		}
		
		return;
	}
	
	//handler takes the kickoff
	if (kickoff && setup && our_action)
	{
		//approach the ball 
		//cannot touch it!!
		
		skill.motion.pos = vision_packet.ball.pos - 
			vision_packet.ball.pos.norm() * .2; 
		skill.motion.face = Point2d(0,5.0);
	}
	else if (running)
	{
		if (_state == GetBall)
		{
			skill.skill = SkillCmd::GotoBall;
			
			skill.motion.face = Point2d(0,5.0);
#if 0
			list<Robot*> forwards;
			Robot::find_by_type<Tactics::Forward*>(forwards);
			
			
			if (forwards.size())
			{
				skill.motion.face = Point2d(0,5.0);
			}
			else
			{
				skill.motion.face = Point2d(0,5.0);
			}
#endif		
			if (robot()->skill_status_code() == SkillStatus::Done)
			{
				robot()->clear_status();
				_state = Pass;
			}
		}
		else if (_state == Pass)
		{
			skill.skill = SkillCmd::Kick;
			
			skill.motion.kick = 255;
			
			//do not change face
			skill.motion.pos = robot()->vision()->pos;
			
			if (robot()->skill_status_code() == SkillStatus::Done)
			{
				robot()->clear_status();
				_state = Done;
			}
		}
		else if (_state == Done)
		{
			skill.valid = false;
		}
	}
	else
	{
		skill.valid = false;
	}
}
