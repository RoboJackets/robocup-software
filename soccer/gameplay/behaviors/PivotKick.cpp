#include "PivotKick.hpp"

#include <stdio.h>

using namespace std;
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
	const float Initial_Accuracy = cos(20 * DegreesToRadians);
	// Accuracy change per frame
	const float Accuracy_Delta = 0.002;
	const float FireNowThreshold = cos(3 * DegreesToRadians);
	
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	// The direction we're facing
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	
	if (robot->hasBall)
	{
		_lastBallTime = state()->timestamp;
	}
	
	// State changes
	Point toBall = (ball().pos - robot->pos).normalized();
	float err = dir.dot(toBall);
	bool behindBall = ((target.center() - robot->pos).dot(ball().pos - robot->pos) > 0);
	if (_state == State_Approach)
	{
		robot->addText(QString().sprintf("err %f", acos(err) * RadiansToDegrees));
		if (robot->hasBall)
		{
			_state = State_Aim;
			_lastError = 0;
			_lastDelta = 0;
			_ccw = ((target.pt[0] - ball().pos).cross(target.pt[1] - ball().pos) > 0);
		} else if (err >= cos(20 * DegreesToRadians) && behindBall)
		{
			_state = State_Capture;
		}
		_accuracy = Initial_Accuracy;
	} else if (_state == State_Capture)
	{
		if (robot->hasBall)
		{
			_state = State_Aim;
			_lastError = 0;
			_lastDelta = 0;
			_ccw = ((target.pt[0] - ball().pos).cross(target.pt[1] - ball().pos) > 0);
		} else if (err < cos(25 * DegreesToRadians) && !behindBall)
		{
			_state = State_Approach;
		}
	} else if (_state == State_Aim)
	{
		if ((!robot->hasBall && (state()->timestamp - _lastBallTime) > 500000) || !ball().pos.nearPoint(robot->pos, Robot_Radius * 1.2))
		{
			_state = State_Approach;
		}
	}
	
	state()->drawLine(ball().pos, target.pt[0], Qt::red);
	state()->drawLine(ball().pos, target.pt[1], Qt::black);
	state()->drawLine(target, Qt::yellow);
	
	// Driving
	robot->dribble(96);
	robot->face(ball().pos);
	if (_state == State_Approach)
	{
		robot->addText("Approach");
		robot->avoidBall(Ball_Radius + 0.05);
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
		state()->drawLine(robot->pos, (ball().pos - robot->pos).normalized() * 8, Qt::green);
		
		// See if it's time to kick
		float error = dir.dot((target.center() - ball().pos).normalized());
		float delta = error - _lastError;
		
		robot->disableAvoidBall();
		if (error >= FireNowThreshold || (error >= _accuracy && _lastDelta > 0 && delta <= 0))
		{
			robot->kick(255);
			robot->addText("KICK");
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
			QString::number(_accuracy)));
		
		_accuracy -= Accuracy_Delta;
		_accuracy = max(0.0f, _accuracy);
		
		// manual pivot code
//		float angle = _ccw ? 10 : -10;
//		robot->move(Point::rotate(robot->pos, ball().pos, angle));
//		robot->face(ball().pos);

		// using pivot motion command
		robot->pivot(ball().pos, _ccw, ball().pos.distTo(robot->pos));

	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
