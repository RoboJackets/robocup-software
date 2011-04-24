#include "PivotKick.hpp"

#include <stdio.h>

using namespace Geometry2d;

Gameplay::Behaviors::PivotKick::PivotKick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	_state = State_Approach;
	_ccw = true;
	
	target.pt[0] = Point(Field_GoalWidth / 2, Field_Length);
	target.pt[1] = Point(-Field_GoalWidth / 2, Field_Length);
}

void Gameplay::Behaviors::PivotKick::restart()
{
	_state = State_Approach;
}

bool Gameplay::Behaviors::PivotKick::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	// The direction we're facing
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	
	// State changes
	Point toBall = (ball().pos - robot->pos).normalized();
	float err = dir.dot(toBall);
	bool facingBall = (err >= cos(20 * DegreesToRadians) && (target.center() - robot->pos).dot(ball().pos - robot->pos) > 0);
	if (_state == State_Approach)
	{
		robot->addText(QString("err %1").arg(err));
		if (robot->hasBall)
		{
			_state = State_Aim;
			_lastError = 0;
			_lastDelta = 0;
			_ccw = ((target.pt[0] - ball().pos).cross(target.pt[1] - ball().pos) > 0);
		} else if (facingBall)
		{
			_state = State_Capture;
		}
	} else if (_state == State_Capture)
	{
		if (robot->hasBall)
		{
			_state = State_Aim;
			_lastError = 0;
			_lastDelta = 0;
			_ccw = ((target.pt[0] - ball().pos).cross(target.pt[1] - ball().pos) > 0);
		} else if (!facingBall)
		{
			_state = State_Approach;
		}
	} else if (_state == State_Aim)
	{
		if (!robot->hasBall)
		{
			_state = State_Approach;
		}
	}
	
	state()->drawLine(ball().pos, target.pt[0], Qt::red);
	state()->drawLine(ball().pos, target.pt[1], Qt::black);
	
	// Driving
	robot->dribble(63);
	robot->face(ball().pos);
	if (_state == State_Approach)
	{
		robot->addText("Approach");
		robot->avoidBall(0.25);
		robot->kick(0);
		robot->move(ball().pos - (target.pt[0] - ball().pos).normalized() * Robot_Radius * 1.3);
		robot->face(ball().pos);
	} else if (_state == State_Capture)
	{
		robot->addText("Capture");
		robot->disableAvoidBall();
		robot->kick(0);
// 		robot->move(ball().pos - (target.pt[0] - ball().pos).normalized() * Robot_Radius);
		robot->move(ball().pos);
		robot->face((ball().pos - robot->pos) * 1.2 + robot->pos);
	} else if (_state == State_Aim)
	{
		state()->drawLine(robot->pos, robot->pos + dir * 8, Qt::white);
		state()->drawLine(ball().pos, target.center(), Qt::yellow);
		
		// See if it's time to kick
		float error = dir.dot((target.center() - ball().pos).normalized());
		float delta = error - _lastError;
		
		robot->disableAvoidBall();
		if (error >= cos(1.5 * DegreesToRadians) || (error >= cos(5 * DegreesToRadians) && _lastDelta > 0 && delta <= 0))
		{
			robot->kick(255);
		} else {
			robot->kick(0);
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
		robot->addText(QString("Aim %1 %2 %3").arg(
			QString::number(acos(error) * RadiansToDegrees),
			QString::number(delta),
			QString::number(_lastDelta)));
		
		float angle = _ccw ? 10 : -10;
		robot->move(Point::rotate(robot->pos, ball().pos, angle));
		robot->face(ball().pos);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
