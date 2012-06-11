#include "Idle.hpp"

#include <boost/foreach.hpp>

using namespace std;

Gameplay::Behaviors::Idle::Idle(GameplayModule *gameplay) :
	Behavior(gameplay)
{
}

bool Gameplay::Behaviors::Idle::run()
{
	if (!ball().valid || robots.empty())
	{
		// If we can't find the ball, leave the robots where they are
		return false;
	}
	
	// Arrange all robots close together around the ball
	
	// We really should exclude parts of the circle that aren't accessible due to field or rules,
	// but I think it doesn't matter when there are only four robots.
	
	// Radius of the circle that will contain the robot centers
	float radius = Field_CenterRadius + Robot_Radius + .01;
	
	// Angle between robots, as seen from the ball
	float perRobot = (Robot_Diameter * 1.25) / radius * RadiansToDegrees;
	
	// Direction from the ball to the first robot.
	// Center the robots around the line from the ball to our goal
	Geometry2d::Point dir = (Geometry2d::Point() - ball().pos).normalized() * radius;
	dir.rotate(Geometry2d::Point(), -perRobot * (robots.size() - 1) / 2);
	
	state()->drawLine(ball().pos, Geometry2d::Point());
	
	BOOST_FOREACH(OurRobot *r, robots)
	{
		if (r->visible)
		{
			state()->drawLine(ball().pos, ball().pos + dir);
			
			r->move(ball().pos + dir);
			r->face(ball().pos);
			
			// Avoid the ball even on our restart
			r->avoidBall(Field_CenterRadius);
			// Don't leave gaps
			r->avoidAllTeammates(true);
		}
		
		// Move to the next robot's position
		dir.rotate(Geometry2d::Point(), perRobot);
	}
	
	return false;
}
