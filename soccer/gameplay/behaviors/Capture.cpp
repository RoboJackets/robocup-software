#include "Capture.hpp"

#include <Utils.hpp>

using namespace std;
using namespace Geometry2d;

static const int Dribble_Speed = 127;

// How far away from the ball the approach point is placed
static const float Approach_Distance = 0.1;
// Ball avoidance distance
static const float Approach_Clearance = 0.1;
// How close we must get to the approach point to proceed to Capture
static const float Approach_Threshold = 0.1;

// How fast we drive towards the ball
static const double Capture_Speed = 0.3;
// How long we must continuously hold the ball to proceed to Aim
static const uint64_t Capture_Time_Threshold = 300 * 1000;
// How much of Capture_Time_Threshold should be spent decelerating
static const double Capture_Decel = 0.8;

// How close the ball must be to count as captured properly
static const double Has_Ball_Dist = 0.1;

// Angular speed for Pivoting
static const double Pivot_Speed = 0.5 * M_PI;

Gameplay::Behaviors::Capture::Capture(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	restart();
	
	target = Point(0, Field_Length); // center of goal
}

void Gameplay::Behaviors::Capture::restart()
{
	_state = State_Approach;
	_ccw = true;
	enable_pivot = true;
}

bool Gameplay::Behaviors::Capture::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	// The direction we're facing
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	
	uint64_t now = Utils::timestamp();
	
	// State changes
	Point toBall = (ball().pos - robot->pos).normalized();
	float err = dir.dot(toBall);
	bool behindBall = ((target - robot->pos).dot(ball().pos - robot->pos) > 0);
	Point approachPoint = ball().pos - (target - ball().pos).normalized() * Approach_Distance;
	if (_state == State_Approach)
	{
		robot->addText(QString("err %1 %2").arg(err).arg(robot->pos.distTo(approachPoint)));
		if (robot->hasBall())
		{
			if (enable_pivot)
			{
				_state = State_Pivoting;
				_ccw = ((target - ball().pos).cross(target - ball().pos) > 0);
			} else
			{
				_state = State_Done;
			}
		} else if (robot->pos.nearPoint(approachPoint, Approach_Threshold) && err >= cos(10 * DegreesToRadians))
		{
			_state = State_Capture;
			_lastBallTime = now;
		}
	} else if (_state == State_Capture)
	{
		// _lastBallTime is the last time we did not have the ball
		if (!robot->hasBall())
		{
			_lastBallTime = now;
		}
		
		if (!behindBall)
		{
			_state = State_Approach;
		}
		
		if ((now - _lastBallTime) >= Capture_Time_Threshold)
		{
			if (!enable_pivot || (ball().pos.nearPoint(robot->pos, Has_Ball_Dist) && err >= cos(20 * DegreesToRadians)))
			{
				_state = State_Done;
			} else {
				_state = State_Pivoting;
			}
			_ccw = dir.cross(target - robot->pos) > 0;
			_lastBallTime = now;
		}
	} else if (_state == State_Pivoting)
	{
		if (!enable_pivot)
		{
			_state = State_Done;
		}

		// _lastBallTime is the last time we had the ball
		if (robot->hasBall())
		{
			_lastBallTime = now;
		}

		if ((!robot->hasBall() && (state()->timestamp - _lastBallTime) > 500000) || !ball().pos.nearPoint(robot->pos, Approach_Distance))
		{
			_state = State_Approach;
		} else if (ball().pos.nearPoint(robot->pos, Has_Ball_Dist) && err >= cos(20 * DegreesToRadians))
		{
			_state = State_Done;
		}
	}
	
	state()->drawLine(ball().pos, target, Qt::red);
	
	// Driving
	if (_state == State_Approach)
	{
		robot->addText("Approach");
		robot->avoidBall(Approach_Clearance);
		robot->move(approachPoint);
		robot->face(ball().pos);
	} else if (_state == State_Capture)
	{
		robot->addText("Capture");
		
		double speed = max(0.0, 1.0 - double(now - _lastBallTime) / double(Capture_Time_Threshold * Capture_Decel)) * Capture_Speed;
		
		robot->dribble(Dribble_Speed);
		robot->worldVelocity(toBall * speed);
		robot->face((ball().pos - robot->pos) * 1.2 + robot->pos);
	} else if (_state == State_Pivoting)
	{
		robot->addText("Pivoting");
		state()->drawLine(robot->pos, robot->pos + dir * 8, Qt::white);
		state()->drawLine(ball().pos, target, Qt::yellow);
		state()->drawLine(robot->pos, (ball().pos - robot->pos).normalized() * 8, Qt::green);

		// See if we've gotten close enough
		float error = dir.dot((target - ball().pos).normalized());

		// Decide which direction to rotate around the ball
		Point rb = ball().pos - robot->pos;
		if (rb.cross(target - ball().pos) > 0)
		{
			_ccw = true;
		} else if ((target - ball().pos).cross(rb) > 0)
		{
			_ccw = false;
		}
		robot->addText(QString("Pivot %1 %2").arg(
			QString::number(acos(error) * RadiansToDegrees),
			QString::number(_ccw ? 1 : 0)));
		
		robot->pivot(Pivot_Speed * (_ccw ? 1 : -1), ball().pos);
		robot->dribble(Dribble_Speed);
	} else {
		robot->addText("Done");
		robot->dribble(Dribble_Speed);
		return false;
	}
	
	return true;
}
