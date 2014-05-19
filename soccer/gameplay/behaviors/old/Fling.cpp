#include "Fling.hpp"

#include <Utils.hpp>

using namespace std;
using namespace Geometry2d;

namespace Gameplay
{
	namespace Behaviors
	{
		REGISTER_CONFIGURABLE(Fling)
	}
}

ConfigDouble *Gameplay::Behaviors::Fling::_fling_travel_thresh;
ConfigDouble *Gameplay::Behaviors::Fling::_pivot_Speed;

void Gameplay::Behaviors::Fling::createConfiguration(Configuration* cfg)
{
	_fling_travel_thresh  = new ConfigDouble(cfg, "Fling/Ball Travel Threshold", 0.5);
	_pivot_Speed  = new ConfigDouble(cfg, "Fling/Pivot Speed", 0.5 * M_PI);
}

Gameplay::Behaviors::Fling::Fling(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay), _capture(gameplay)
{
	target = Point(0.0, Field_Length);
	restart();
}

void Gameplay::Behaviors::Fling::restart()
{
	_state = State_Capture;
	_capture.restart();
	enable_fling = true;
	dribble_speed = 50;
	_ccw = true;
}

bool Gameplay::Behaviors::Fling::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	// force sub-behavior to have its robot assigned
	_capture.robot = robot;

	// The direction we're facing
	const Point dir = Point::direction(robot->angle * DegreesToRadians);

	// Geometry
	// Point toBall = (ball().pos - robot->pos).normalized();
	Segment flingLine(target, ball().pos);
	Point toTarget = flingLine.delta().normalized();
	float err = dir.dot(toTarget); // we want to be perpendicular to the fling line
	
	// State changes
	if (_state == State_Capture)
	{
		if (_capture.done())
		{
			_state = State_Pivot;
			_ccw = dir.cross((target - robot->pos).normalized().perpCW()) > 0; // FIXME check direction
		}
	} else if (_state == State_Pivot)
	{
		if (!robot->hasBall())
		{
			_state = State_Capture;
		}

		if (enable_fling && err <= cos(20 * DegreesToRadians))
		{
			// if we are close enough to perpendicular to the fling line
			_state = State_Fling;
		}
	} else if (_state == State_Fling)
	{
		// we are done when the ball has moved
		if (!ball().pos.nearPoint(_flingBallStart, *_fling_travel_thresh))
		{
			_state = State_Done;
		}
	}
	
	state()->drawLine(flingLine, Qt::red);
	
	// Driving
	if (_state == State_Capture)
	{
		robot->addText("Capturing");
		_capture.run();
	}  else if (_state == State_Pivot)
	{
		robot->addText("Pivoting");
		state()->drawLine(robot->pos, robot->pos + dir * 8, Qt::white);
		state()->drawLine(ball().pos, target, Qt::yellow);
		state()->drawLine(robot->pos, (ball().pos - robot->pos).normalized() * 8, Qt::green);

		// See if we've gotten close enough
		float error = dir.dot((target - ball().pos).normalized().perpCW());

		// Decide which direction to rotate around the ball
		Point rb = ball().pos - robot->pos;
		if (rb.cross((target - ball().pos).normalized().perpCW()) > 0)
		{
			_ccw = true;
		} else if ((target - ball().pos).normalized().perpCW().cross(rb) > 0)
		{
			_ccw = false;
		}
		robot->addText(QString("Pivot %1 %2").arg(
			QString::number(acos(error) * RadiansToDegrees),
			QString::number(_ccw ? 1 : 0)));

		robot->pivot(*_pivot_Speed * (_ccw ? 1 : -1), ball().pos);
		robot->dribble(dribble_speed);

	}  else if (_state == State_Fling)
	{
		robot->addText("Flinging");
		robot->dribble(dribble_speed);
		
		// choose the direction of the spin
		float w = (robot->pointInRobotSpace(target).y > 0) ? spin_speed : -1.0 * spin_speed;
		
		// robot->angularVelocity(w);
		//	FIXME: when angularVelocity() was removed, this behavior got broken :(
		//		if someone would show it some love and use the new motion commands here, that'd be great!

	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
