#include "Idle.hpp"

#include <boost/foreach.hpp>

Gameplay::Behaviors::Idle::Idle(GameplayModule *gameplay) :
	Behavior(gameplay)
{
}

void Gameplay::Behaviors::Idle::run()
{
	// Avoid the ball even on our restart
	robot()->avoidBall = true;
	
	const Packet::LogFrame::Ball& b = ball();
	
	if (b.valid)
	{
		//robot()->state()->cmd.goalPosition = ball;
		std::list<Robot*> allIdle;
		gameplay()->find_by_type<Gameplay::Behaviors::Idle*>(allIdle);
		
		const float radius = .5 + Constants::Robot::Radius + .01;
		
		//angle in radians * radius = arcLength
		
		const float totalArc = allIdle.size() * (Constants::Robot::Diameter + .04);
		
		float totalSpan = totalArc/radius * RadiansToDegrees;
		float perRobot = totalSpan/allIdle.size();
		
		Geometry2d::Point dir = (Geometry2d::Point() - b.pos).normalized() * radius;
		dir.rotate(Geometry2d::Point(), -totalSpan/2.0 + perRobot/2.0);
		
		BOOST_FOREACH(Robot* r, allIdle)
		{
			if (r->id() == robot()->id())
			{
				break;
			}
				
			dir.rotate(Geometry2d::Point(), perRobot);
		}
		
		robot()->move(b.pos + dir);
		robot()->face(b.pos);
	}
}

float Gameplay::Behaviors::Idle::score(Robot* r)
{
	return r->pos().distTo(ball().pos);
}
