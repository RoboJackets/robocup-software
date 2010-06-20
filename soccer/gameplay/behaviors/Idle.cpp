#include "Idle.hpp"

#include <boost/foreach.hpp>

using namespace std;

Gameplay::Behaviors::Idle::Idle(GameplayModule *gameplay) :
	Behavior(gameplay, 1)
{
}

bool Gameplay::Behaviors::Idle::assign(std::set< Gameplay::Robot* >& available)
{
    return takeAll(available);
}

bool Gameplay::Behaviors::Idle::run()
{
	if (!ball().valid)
	{
		// If we can't find the ball, leave the robots where they are
		return false;
	}
	
	// Arrange all robots close together around the ball
	
	// We really should exclude parts of the circle that aren't accessible due to field or rules,
	// but I think it doesn't matter when there are only four robots.
	
	// Radius of the circle that will contain the robot centers
	float radius = Constants::Field::CenterRadius + Constants::Robot::Radius + .01;
	
	// Angle between robots, as seen from the ball
	float perRobot = (Constants::Robot::Diameter * 1.25) / radius * RadiansToDegrees;
	
	// Direction from the ball to the first robot.
	// Center the robots around the line from the ball to our goal
	Geometry2d::Point dir = (Geometry2d::Point() - ball().pos).normalized() * radius;
	dir.rotate(Geometry2d::Point(), -perRobot * (_robots.size() - 1) / 2);
	
	Packet::LogFrame::DebugLine line;
	line.pt[0] = ball().pos;
	line.pt[1] = Geometry2d::Point();
	// make line black
	line.color[0] = 0.0; line.color[1] = 0.0; line.color[2] = 0.0;
	_gameplay->state()->debugLines.push_back(line);
	BOOST_FOREACH(Robot *r, _robots)
	{
		if (r->visible())
		{
			Packet::LogFrame::DebugLine velLine;
			velLine.pt[0] = ball().pos;
			velLine.pt[1] = ball().pos + dir;
			// make line black
			velLine.color[0] = 0.0; velLine.color[1] = 0.0; velLine.color[2] = 0.0;
			_gameplay->state()->debugLines.push_back(velLine);
			
			r->move(ball().pos + dir);
			r->face(ball().pos);
			
			// Avoid the ball even on our restart
			r->avoidBall = true;
		}
		
		// Move to the next robot's position
		dir.rotate(Geometry2d::Point(), perRobot);
	}
	
	return false;
}
