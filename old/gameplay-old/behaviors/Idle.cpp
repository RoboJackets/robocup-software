#include "Idle.hpp"



using namespace std;

Gameplay::Behaviors::Idle::Idle(GameplayModule *gameplay) :
	Behavior(gameplay)
{
}

Gameplay::Behaviors::Idle::~Idle()
{
    for (OurRobot* r :  robots)
    {
        if(r)
            r->resetAvoidBall();
    }
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
	
	for (OurRobot *r :  robots)
	{
		if (r->visible)
		{
			state()->drawLine(ball().pos, ball().pos + dir);
			
			r->move(ball().pos + dir);
			r->face(ball().pos);
			
			// Avoid the ball even on our restart
			r->avoidBallRadius(Field_CenterRadius);
			// Don't leave gaps
			r->avoidAllTeammates(true);
		}
		
		// Move to the next robot's position
		dir.rotate(Geometry2d::Point(), perRobot);
	}
	
	return false;
}

bool Gameplay::Behaviors::Idle::run(const Geometry2d::Segment& line)
{
	if (robots.empty())
	{
		// Leave the robots where they are
		return false;
	}

	float length = line.length();

	if (length/(float)robots.size() < Robot_Diameter)
	{
		// Can't fit on line
		return false;
	}

	Geometry2d::Point loc = line.pt[0];

	Geometry2d::Point incr = line.delta().normalized() * length / (float) robots.size();

	for (OurRobot* r :  robots)
	{
		r->move(loc);
		loc += incr;
	}

	return false;
}
