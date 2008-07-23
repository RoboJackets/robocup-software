#include "Goalie.hpp"
#include "Assistant_Goalie.hpp"

#include <boost/foreach.hpp>

#include <Sizes.h>
#include <Geometry/Line2d.hpp>

#include "../Predicates.hpp"

using namespace std;
using namespace Geometry;
using namespace Packet;

Tactics::Goalie::Goalie(): Base(0)
{
}

float Tactics::Goalie::score(Robot *robot)
{
    // Pick the robot closest to the goal.
    //return robot->vision()->pos.magsq();
	
	//gimped robot, use id 0
	return robot->id();
}

void Tactics::Goalie::run()
{
	//handle penalty kick
	if (penalty && !our_action)
	{
		float destX = 0;
		
		//need to be on goal line
		Line2d baseLine(Point2d(-1, ROBOT_RADIUS), Point2d(1, ROBOT_RADIUS));
		
		//get the striker opponent
		Robot* r = Robot::find("striker");
		if (r)
		{

			Line2d dir(r->pos(), vision_packet.ball.pos);
			
			Point2d intersect;
			if (baseLine.intersects(dir, &intersect) 
					&& fabs(intersect.x) < GOAL_WIDTH/2.0)
			{
				destX = intersect.x;
			}
		}
		
		Packet::SkillCmd::Robot& skill = *robot()->skill();
		skill.skill = Packet::SkillCmd::None;
		skill.valid = true;
		//skill.motion.avoid = false;
		//skill.motion.style = MotionCmd::Fast;
		skill.motion.face = vision_packet.ball.pos;
		skill.motion.pos.x = destX;
		skill.motion.pos.y = ROBOT_RADIUS - .02;
		
		return;
	}
	
	if (!vision_packet.ball.valid)
	{
		//mm...don't change position...
		return;
	}
	
	//get assistant goalies
	list<Robot*> assistants;
	Robot::find_by_type<Tactics::Assistant_Goalie*>(assistants);
	//printf("Assistants: %d\n", (int)assistants.size());

	//if true the goalie will be used like an assistant
	bool useSelf = true;

	//start with full goal
	list<Tactics::Goalie::Span> spans;
	spans.push_back(Span(-GOAL_WIDTH/2.0, GOAL_WIDTH/2.0));

	const Point2d bPos = vision_packet.ball.pos;
	const Point2d bVel = vision_packet.ball.vel;

	//shot on goal detection for goalie
	//need to do something else with assistants
	Point2d inter;

	Line2d bPath(bPos, bPos + bVel);
	Line2d goalLine(Point2d(-1, ROBOT_RADIUS), Point2d(1, ROBOT_RADIUS));

	if (bPath.intersects(goalLine, &inter) && bVel.mag() > 2.0f && fabs(inter.x) < GOAL_WIDTH/2.0)
	{
		//ball is shot
		//get on goal line to stop it
		robot()->move(inter);
		robot()->face(bPos);

		//goalie will not be used as an assistant
		useSelf = false;

		//remove his current shadow
		Circle2d r(robot()->pos(), ROBOT_RADIUS);
		Tactics::Goalie::removeShadow(Tactics::Goalie::shadow(r, bPos), spans);
	}

	//use goalie as an assistant
	if (useSelf)
	{
		assistants.push_back(robot());
	}

	unsigned int assistantCount = assistants.size();

	//divide up the goal into N equal parts
	Span goal(-GOAL_WIDTH/2.0, GOAL_WIDTH/2.0);
	const float div = goal.size() / assistantCount;

	float x = -GOAL_WIDTH/2.0 + div/2.0;

	//each assistant will go in the center of a part
	for (unsigned int i=0 ; i <assistantCount ; ++i)
	{
		//line from div to ball
		Line2d bisector(Point2d(x, 0), bPos);

		//robot to use
		Robot* use = assistants.front();

		//pick closest robot to destination
		BOOST_FOREACH(Robot* r, assistants)
		{
			const float dist = bisector.distTo(r->pos());

			if (dist < bisector.distTo((use->pos())))
			{
				use = r;
			}
		}

		assistants.remove(use);

		//circle radius around 0,0
		float dist = .35;

		Packet::SkillCmd::Robot& skill = *use->skill();
		skill.skill = Packet::SkillCmd::None;
		
		if (stopped | setup)
		{
			skill.motion.style = MotionCmd::Accurate;
		}
		{
			skill.motion.style = MotionCmd::Fast;
		}

		if (use != robot())
		{
			dist = .75;
			use->free(false);

			//set goalie avoid zone for assistants
			skill.motion.avoid = true;
			skill.motion.avoidZone = Circle2d(Point2d(0,0), dist - ROBOT_RADIUS - .01);
		}
		else
		{
			skill.motion.avoid = false;
			skill.motion.goalie = true;
		}

		Circle2d arc(Point2d(0,0), dist);
		Point2d p1;

		if (bisector.intersects(arc, &p1))
		{
			use->move(p1);
			use->face(bPos);
		}

		x += div;
	}
}

std::list<Tactics::Goalie::Span> Tactics::Goalie::calcWindow(int ignore)
{
	std::list<Goalie::Span> windows;

	if (vision_packet.ball.valid)
	{
		//start with fully open goal
		windows.push_back(Goalie::Span(-GOAL_WIDTH/2.0f, GOAL_WIDTH/2.0f));

		const Point2d bPos = vision_packet.ball.pos;

		//for each obstacle, create span
		for (int i=0 ; i<5 ; ++i)
		{
			if (ignore != i && vision_packet.self[i].valid)
			{
				Circle2d r(vision_packet.self[i].pos, ROBOT_RADIUS);
				removeShadow(shadow(r, bPos), windows);
			}

			if (vision_packet.opp[i].valid)
			{
				Circle2d r(vision_packet.opp[i].pos, ROBOT_RADIUS);
				removeShadow(shadow(r, bPos), windows);
			}
		}

		//TODO is span size < ball diam...remove it
	}

	return windows;
}

/** remove the shadow from pevious windows */
void Tactics::Goalie::removeShadow(Goalie::Span s, std::list<Goalie::Span>& spans)
{
	if (s.min == s.max)
	{
		return;
	}

	list<Span>::iterator i = spans.begin(), next;
	for (; i!= spans.end() ; i=next)
	{
		next = i;
		++next;

		//do subtraction
		Span& old = *i;

		if (s.min < old.min && s.max < old.max && s.max > old.min)
		{
			//change low
			old.min = s.max;
		}
		else if (s.min > old.min && s.min < old.max && s.max > old.max)
		{
			//change high
			old.max = s.min;
		}
		else if (s.min > old.min && s.max < old.max)
		{
			//split
			Span n(s.max, old.max);
			old.max = s.min;

			spans.push_back(n);
		}
		else if (s.min < old.min && s.max > old.max)
		{
			//remove
			spans.erase(i);
		}
	}
}

/** see if the robot casts a shadow onto the goal */
Tactics::Goalie::Span Tactics::Goalie::shadow(Geometry::Circle2d obstacle, Geometry::Point2d orig)
{
	Point2d p1, p2;

	if (obstacle.y() < orig.y && obstacle.tangentPoints(orig, &p1, &p2))
	{
		float slope1 = (orig.y - p1.y)/(orig.x - p1.x);
		float b1 = orig.y - slope1 * orig.x;
		float x1 = -b1/slope1;

		float slope2 = (orig.y - p2.y)/(orig.x - p2.x);
		float b2 = orig.y - slope2 * orig.x;
		float x2 = -b2/slope2;

		if (x1 < x2)
		{
			return Span(x1, x2);
		}
		else
		{
			return Span(x2, x1);
		}
	}

	return Span(0,0);
}
