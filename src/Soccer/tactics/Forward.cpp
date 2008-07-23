#include "Forward.hpp"
#include "../Role.hpp"

#include <list>
#include <boost/foreach.hpp>

#include "../Predicates.hpp"
#include "parameters/Robot_Parameter.hpp"

#include <Sizes.h>
#include <Geometry/Point2d.hpp>
#include <Geometry/Line2d.hpp>
#include <Geometry/Circle2d.hpp>

using namespace std;
using namespace Geometry;
using namespace Packet;

Tactics::Factory_Type<Tactics::Forward> forward("forward");

Tactics::Forward::Forward(Role *role) :
    Base(role),
    _opp(this, "opp")
{
}

void Tactics::Forward::run()
{
	//we have an op
	//no mark, sit between him and closest goal post
	if (_opp.valid() && kickoff && (setup | waiting))
	{
		Packet::SkillCmd::Robot& skill = *robot()->skill();
		skill.valid = true;
		skill.skill = Packet::SkillCmd::None;
		
		Point2d oppPos = vision_packet.opp[_opp.robot()->id()].pos;
		
		float xCoord = -GOAL_WIDTH/2.0;
		
		if (fabs(oppPos.x - xCoord) > fabs(oppPos.x - GOAL_WIDTH/2.0))
		{
			xCoord = GOAL_WIDTH/2.0;
		}
		
		Line2d shotLine(oppPos, Point2d(xCoord, 0));
		Line2d travelLine(Point2d(-1, FIELD_LENGTH/2.0 - ROBOT_RADIUS),
				Point2d(1, FIELD_LENGTH/2.0 - ROBOT_RADIUS));
		
		Point2d dest;
		
		if (shotLine.intersects(travelLine, &dest))
		{
			skill.motion.pos = dest;
			skill.motion.face = oppPos;
		}
		else
		{
			skill.skill = Packet::SkillCmd::Mark;
			skill.motion.style = MotionCmd::Fast;
			skill.markID = _opp.robot()->id();
		}
		
		//avoid the ball on their kickoff
		skill.motion.avoid = true;
		skill.motion.avoidZone = Circle2d(vision_packet.ball.pos, .5);
		
		return;
	}
	
	//create possible location zones
	list<Circle2d> zones;
	zones.push_back(Circle2d(1.5, 1.5, 0.5));
	zones.push_back(Circle2d(-1.5, 1.5, 0.5));
	
	//pick a zone to go into
	if (robot()->free() && vision_packet.ball.valid && our_action 
			&& kickoff && (setup | running))
	{
		list<Robot*> friends;
		Robot::find_by_type<Tactics::Forward*>(friends);
		
		BOOST_FOREACH(Circle2d zone, zones)
		{
			if (friends.empty())
			{
				return;
			}
			
			Robot* use = friends.front();
			
			//TODO handle no robots left
			BOOST_FOREACH(Robot* r, friends)
			{
				r->free(true);
				
				if (r->pos().distTo(zone.center()) < 
						use->pos().distTo(zone.center()))
				{
					use = r;
				}
			}
			
			friends.remove(use);
			
			//goto center of zone
			if (use != robot())
			{
				use->free(false);
			}
			
			Packet::SkillCmd::Robot& skill = *use->skill();
			skill.valid = true;
			skill.skill = Packet::SkillCmd::None;
			//skill.motion.style = MotionCmd::Fast;
			skill.motion.pos = zone.center();
			skill.motion.face = vision_packet.ball.pos;
		}
		
		return;
	}
	
	//gather around the ball
	if (robot()->free() && (stopped || setup) && vision_packet.ball.valid)
	{
		list<Robot*> friends;
		Robot::find_by_type<Tactics::Forward*>(friends);

		const int friendCount = friends.size();
		const float rAngle = 30;

		float angle = -(friendCount*rAngle)/2.0 + rAngle/2.0;

		const Point2d bPos = vision_packet.ball.pos;
		Point2d dest = bPos + (Point2d(0,0) - bPos).norm() * (.5 + ROBOT_RADIUS);

#if 0
		if(indirect_kick)
		{
			if(opp_param.valid())
			{
				Point2d opp_pos = opp_param.robot()->pos();
				Line2d opp_ball_line(bPos,opp_pos);
				Point2d dest = opp_ball_line.nearest_point(dest);
			}
		}
#endif

		dest.rotate(bPos, angle);

		for(int i=0 ; i<friendCount ; ++i)
		{
			Robot* use = friends.front();

			//given dest, find best robot
			BOOST_FOREACH(Robot* r, friends)
			{
				r->free(true);

				if (r->pos().distTo(dest) < use->pos().distTo(dest))
				{
					use = r;
				}
			}

			friends.remove(use);

			if (use != robot())
			{
				use->free(false);
			}

			Packet::SkillCmd::Robot& skill = *use->skill();
			skill.valid = true;
			skill.skill = Packet::SkillCmd::None;
			skill.motion.style = MotionCmd::Accurate;

			use->move(dest);
			use->face(bPos);

			dest.rotate(bPos, rAngle);
		}
		
		return;
	}
}
