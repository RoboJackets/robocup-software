#include "Capture.hpp"

#include <framework/RobotConfig.hpp>
#include <Utils.hpp>

using namespace std;
using namespace Geometry2d;

Gameplay::Behaviors::Capture::Capture(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	restart();
	
	target = Point(0, Field_Length); // center of goal

	_stationaryMaxSpeed = config()->createDouble("Capture/Ball Speed Threshold", 0.5);
	_approach_Distance = config()->createDouble("Capture/Approach Distance", 0.1);
	_approach_Clearance = config()->createDouble("Capture/Approach Clearance", 0.05);
	_approach_Threshold = config()->createDouble("Capture/Approach Threshold", 0.13);
	_capture_Speed = config()->createDouble("Capture/Capture Speed", 0.3);
	_capture_Time_Threshold = config()->createDouble("Capture/Capture Time Threshold", 300 * 1000);
	_capture_Decel = config()->createDouble("Capture/Capture Decel", 0.8);
	_has_Ball_Dist = config()->createDouble("Capture/Has Ball Distance", 0.1);
	_pivot_Speed = config()->createDouble("Capture/Pivot Speed", 0.5 * M_PI);
	_dribble_Speed = config()->createDouble("Capture/Dribbler Speed", 127);
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
	float ballDist = ball().pos.distTo(robot->pos);
	float err = dir.dot(toBall);
	float ballSpeed = ball().vel.mag();
	bool behindBall = ((target - robot->pos).dot(ball().pos - robot->pos) > 0);

	// Target positioning for robot to trap the ball - if moving,
	// stop ball first, otherwise
	Point targetApproachPoint = ball().pos - (target - ball().pos).normalized() * *_approach_Distance;
	Point approachPoint = targetApproachPoint;

	// pick target based on velocity
	if (ballSpeed > *_stationaryMaxSpeed)
	{
		float interceptTime = ballDist / *robot->config->trapTrans.velocity;
		Point trapApproachPoint = ball().pos + ball().vel * interceptTime; // TODO: check if accel term is necessary
		Point approachPoint = trapApproachPoint;
	}

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
		} else if (robot->pos.nearPoint(approachPoint, *_approach_Threshold) && err >= cos(10 * DegreesToRadians))
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
		
		if ((now - _lastBallTime) >= *_capture_Time_Threshold)
		{
			if (!enable_pivot || (ball().pos.nearPoint(robot->pos, *_has_Ball_Dist) && err >= cos(20 * DegreesToRadians)))
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

		if ((!robot->hasBall() && (state()->timestamp - _lastBallTime) > 500000) || !ball().pos.nearPoint(robot->pos, *_approach_Distance))
		{
			_state = State_Approach;
		} else if (ball().pos.nearPoint(robot->pos, *_has_Ball_Dist) && err >= cos(20 * DegreesToRadians))
		{
			_state = State_Done;
		}
	}
	
	state()->drawLine(ball().pos, target, Qt::red);
	
	// Driving
	if (_state == State_Approach)
	{
		robot->addText("Approach");
		robot->avoidBall(*_approach_Clearance);
		robot->move(approachPoint);
		robot->face(ball().pos);
	} else if (_state == State_Capture)
	{
		robot->addText("Capture");
		
		double speed = max(0.0, 1.0 - double(now - _lastBallTime) / double(*_capture_Time_Threshold * *_capture_Decel)) * *_capture_Speed;
		
		robot->dribble(*_dribble_Speed);
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
		
		robot->pivot(*_pivot_Speed * (_ccw ? 1 : -1), ball().pos);
		robot->dribble(*_dribble_Speed);
	} else {
		robot->addText("Done");
		robot->dribble(*_dribble_Speed);
		return false;
	}
	
	return true;
}
