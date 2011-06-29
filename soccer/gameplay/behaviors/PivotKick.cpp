#include "PivotKick.hpp"

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

// Angular speed for aiming
static const double Aim_Speed = 0.5 * M_PI;

// If the ball is this far away from the robot after trying to kick, then
// the ball was kicked.
static const double Kick_Completed_Threshold = 0.5;

static const float Initial_Accuracy = cos(10 * DegreesToRadians);
// Accuracy change per frame
static const float Accuracy_Delta = 0.000;
static const float FireNowThreshold = cos(3 * DegreesToRadians);

Gameplay::Behaviors::PivotKick::PivotKick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	restart();
	
	target.pt[0] = Point(Field_GoalWidth / 2, Field_Length);
	target.pt[1] = Point(-Field_GoalWidth / 2, Field_Length);
}

void Gameplay::Behaviors::PivotKick::restart()
{
	_state = State_Approach;
	_kicked = false;
	_ccw = true;
}

bool Gameplay::Behaviors::PivotKick::run()
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
	bool behindBall = ((target.center() - robot->pos).dot(ball().pos - robot->pos) > 0);
	Point approachPoint = ball().pos - (target.pt[0] - ball().pos).normalized() * Approach_Distance;
	if (_state == State_Approach)
	{
		robot->addText(QString("err %1 %2").arg(err).arg(robot->pos.distTo(approachPoint)));
		if (robot->hasBall)
		{
			_state = State_Aim;
			_lastError = 0;
			_lastDelta = 0;
			_ccw = ((target.pt[0] - ball().pos).cross(target.pt[1] - ball().pos) > 0);
		} else if (robot->pos.nearPoint(approachPoint, Approach_Threshold) && err >= cos(10 * DegreesToRadians))
		{
			_state = State_Capture;
			_lastBallTime = now;
		}
		_accuracy = Initial_Accuracy;
	} else if (_state == State_Capture)
	{
		// _lastBallTime is the last time we did not have the ball
		if (!robot->hasBall)
		{
			_lastBallTime = now;
		}
		
		if (!behindBall)
		{
			_state = State_Approach;
		}
		
		if ((now - _lastBallTime) >= Capture_Time_Threshold)
		{
			_state = State_Aim;
			_lastError = 0;
			_lastDelta = 0;
			_ccw = dir.cross(target.center() - robot->pos) > 0;
			_lastBallTime = now;
		}
	} else if (_state == State_Aim)
	{
		// _lastBallTime is the last time we had the ball
		if (robot->hasBall)
		{
			_lastBallTime = now;
		}
		
		if ((!robot->hasBall && (state()->timestamp - _lastBallTime) > 500000) || !ball().pos.nearPoint(robot->pos, Kick_Completed_Threshold))
		{
			if (_kicked)
			{
				_state = State_Done;
			} else {
				_state = State_Approach;
			}
		}
	}
	
	state()->drawLine(ball().pos, target.pt[0], Qt::red);
	state()->drawLine(ball().pos, target.pt[1], Qt::black);
	state()->drawLine(target, Qt::yellow);
	
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
	} else if (_state == State_Aim)
	{
#if 0
		state()->drawLine(robot->pos, robot->pos + dir * 8, Qt::white);
		state()->drawLine(ball().pos, target.center(), Qt::yellow);
		state()->drawLine(robot->pos, (ball().pos - robot->pos).normalized() * 8, Qt::green);
		
		// See if it's time to kick
		float error = dir.dot((target.center() - ball().pos).normalized());
		float delta = error - _lastError;
		
		if (error >= FireNowThreshold || (error >= _accuracy && _lastDelta > 0 && delta <= 0))
		{
			robot->kick(255);
			robot->addText("KICK");
			_kicked = true;
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
		
		_accuracy -= Accuracy_Delta;
		_accuracy = max(0.0f, _accuracy);

		robot->pivot(Aim_Speed * (_ccw ? 1 : -1), ball().pos);
#endif
		robot->dribble(Dribble_Speed);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
