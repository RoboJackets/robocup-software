#include "PivotKick.hpp"

#include <Utils.hpp>

using namespace std;
using namespace Geometry2d;

Gameplay::Behaviors::PivotKick::PivotKick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay), _capture(gameplay)
{
	restart();
	
	target.pt[0] = Point(Field_GoalWidth / 2, Field_Length);
	target.pt[1] = Point(-Field_GoalWidth / 2, Field_Length);
	_capture.target = target.pt[0];

	_aim_Speed = config()->createDouble("PivotKick/Aim Speed", 0.5 * M_PI);
	_kick_Completed_Threshold = config()->createDouble("PivotKick/Kick Completed Threshold", 0.5);
	_initial_Accuracy = config()->createDouble("PivotKick/Initial Accuracy", cos(10 * DegreesToRadians));
	_accuracy_Delta = config()->createDouble("PivotKick/Accuracy Delta", 0.000);
	_fireNowThreshold = config()->createDouble("PivotKick/Fire Now Threshold", cos(3 * DegreesToRadians));
}

void Gameplay::Behaviors::PivotKick::restart()
{
	_state = State_Capture;
	_kicked = false;
	_ccw = true;
	_capture.restart();
	_capture.target = target.pt[0];
	enable_kick = true;
	enable_desparate_kick = true;
	dribble_speed = 50;
	use_chipper = false;
	kick_power = 255;
}

bool Gameplay::Behaviors::PivotKick::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	// force sub-behavior to have its robot assigned
	_capture.robot = robot;

	// The direction we're facing
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	
	uint64_t now = Utils::timestamp();
	
	// State changes
	Point toBall = (ball().pos - robot->pos).normalized();
	if (_state == State_Capture)
	{
		if (_capture.done())
		{
			_state = State_Aim;
			_lastError = 0;
			_lastDelta = 0;
			_ccw = ((target.pt[0] - ball().pos).cross(target.pt[1] - ball().pos) > 0);
		}
		_accuracy = *_initial_Accuracy;
	} else if (_state == State_Aim)
	{
		// _lastBallTime is the last time we had the ball
		if (robot->hasBall())
		{
			_lastBallTime = now;
		}
		
		// watch for ball leaving the robot
		if ((!robot->hasBall() && (state()->timestamp - _lastBallTime) > 500000) || !ball().pos.nearPoint(robot->pos, *_kick_Completed_Threshold))
		{
			if (_kicked)
			{
				_state = State_Done;
			} else {
				_state = State_Capture;
				_capture.restart();
			}
		}
	}
	
	state()->drawLine(ball().pos, target.pt[0], Qt::red);
	state()->drawLine(ball().pos, target.pt[1], Qt::black);
	state()->drawLine(target, Qt::yellow);
	
	// Driving
	if (_state == State_Capture)
	{
		robot->addText("Capturing");
		_capture.target = target.pt[0];
		_capture.run();
	}  else if (_state == State_Aim)
	{
		state()->drawLine(robot->pos, robot->pos + dir * 8, Qt::white);
		state()->drawLine(ball().pos, target.center(), Qt::yellow);
		state()->drawLine(robot->pos, (ball().pos - robot->pos).normalized() * 8, Qt::green);
		
		// See if it's time to kick
		float error = dir.dot((target.center() - ball().pos).normalized());
		float delta = error - _lastError;
		
		if (enable_kick && (error >= *_fireNowThreshold || (error >= _accuracy && _lastDelta > 0 && delta <= 0)))
		{
			if (use_chipper)
			{
				robot->chip(kick_power);
				robot->addText("CHIP");
			} else
			{
				robot->kick(kick_power);
				robot->addText("KICK");
			}
			_kicked = true;
		} else {
			robot->addText("Aim");
		}
		
		_lastError = error;
		_lastDelta = delta;
		
		// Decide which direction to rotate around the ball
		Point rb = ball().pos - robot->pos;
		if (rb.cross(target.pt[0] - ball().pos) > 0)
		{
			_ccw = true;
		} else if ((target.pt[1] - ball().pos).cross(rb) > 0)
		{
			_ccw = false;
		}
		robot->addText(QString("Aim %1 %2 %3 %4").arg(
			QString::number(acos(error) * RadiansToDegrees),
			QString::number(delta),
			QString::number(_accuracy),
			QString::number(_ccw ? 1 : 0)));
		
		_accuracy -= *_accuracy_Delta;
		_accuracy = max(0.0f, _accuracy);

		robot->pivot(*_aim_Speed * (_ccw ? 1 : -1), ball().pos);
		robot->dribble(dribble_speed);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
