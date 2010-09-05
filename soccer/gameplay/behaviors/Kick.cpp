#include "Kick.hpp"

#include <stdio.h>
#include <algorithm>

using namespace std;

Gameplay::Behaviors::Kick::Kick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	setTargetGoal();
	
	restart();
}

bool Gameplay::Behaviors::Kick::run()
{
	if (!robot || !robot->visible || !ball().valid)
	{
		return false;
	}
	
	Geometry2d::Point ballPos = ball().pos;
	
	Geometry2d::Segment target = _target;
	// Some calculations depend on the order of the target endpoints.
	// Ensure that t0 x t1 > 0.
	// We have to do this each frame since the ball may move to the other side of the target.
	//FIXME - Actually, doesn't that mean the kick/pass is done?
	//FIXME - What about kicking towards a point?  cross product is zero...
	if ((target.pt[0] - ballPos).cross(target.pt[1] - ballPos) < 0)
	{
		swap(target.pt[0], target.pt[1]);
	}
	
	// State transitions
	switch (_state)
	{
		case State_Approach1:
			if (robot->pos.nearPoint(ballPos, Robot_Radius + Ball_Radius + 0.08))
			{
				_state = State_Approach2;
			}
			break;
		
		case State_Approach2:
			if (robot->hasBall)
			{
				robot->addText("Aim");
				_state = State_Aim;
				_lastError = INFINITY;
			}
			break;
		
		case State_Aim:
			if (!robot->hasBall)
			{
				_state = State_Approach2;
			} else {
				state()->drawLine(ballPos, target.pt[0], Qt::red);
				state()->drawLine(ballPos, target.pt[1], Qt::white);
				
				Geometry2d::Point rd = Geometry2d::Point::direction(robot->angle * DegreesToRadians);
				state()->drawLine(robot->pos, robot->pos + rd * Field_Length);
				state()->drawLine(robot->pos, target.center());
				float error = acos(rd.dot((target.center() - robot->pos).normalized())) * RadiansToDegrees;
				robot->addText(QString("Aim %1").arg(error));
				
				if (!isinf(_lastError))
				{
					//FIXME - Depends on t0 x t1 > 0
					bool inT0 = (target.pt[0] - ballPos).cross(rd) > 0;
					bool inT1 = rd.cross(target.pt[1] - ballPos) > 0;
					robot->addText(QString("in %1 %2").arg(inT0).arg(inT1));
					
					//FIXME - Predict the error at the time of kicking (need latency/dwdt measurement)
					// and get rid of the magic number
					if ((error > _lastError && inT0 && inT1) || error < 1.5)
					{
						// Past the best position
						_state = State_Kick;
					}
				}
				_lastError = error;
			}
			break;
		
		case State_Kick:
			if (!robot->hasBall)
			{
				_state = State_Done;
			}
			break;
		
		case State_Done:
			break;
	}
	
	switch (_state)
	{
		case State_Approach1:
			robot->addText("Approach1");
			robot->move(ballPos + (ballPos - target.pt[0]).normalized() * (Robot_Radius + Ball_Radius));
			robot->face(target.pt[0]);
			robot->avoidBall = true;
			break;
			
		case State_Approach2:
			robot->addText("Approach2");
			robot->move(ballPos);
			robot->face(ballPos);
			robot->dribble(127);
			break;
			
		case State_Aim:
		{
			Geometry2d::Point targetCenter = target.center();
			
			// Vector from ball to center of target
			Geometry2d::Point toTarget = targetCenter - ballPos;
			
			// Robot position relative to the ball
			Geometry2d::Point relPos = robot->pos - ballPos;
			
			// True if the robot is in front of the ball
			bool inFrontOfBall = toTarget.perpCCW().cross(relPos) > 0;
			
			MotionCmd::PivotType dir;
			if (inFrontOfBall)
			{
				// Move behind the ball
				dir = (toTarget.cross(relPos) > 0) ? MotionCmd::CCW : MotionCmd::CW;
			} else {
				// Behind the ball: move to the nearest line containing the ball and a target endpoint.
				// Note that the robot is behind the ball, while the target vectors all point in *front* of the ball.
				//FIXME - These assume t0 x t1 > 0.  Enforce this here or above.
				if (toTarget.cross(relPos) > 0)
				{
					// Below the center line: nearest endpoint-line includes target.pt[0]
					dir = (target.pt[0] - ballPos).cross(relPos) > 0 ? MotionCmd::CW : MotionCmd::CCW;
				} else {
					// Above the center line: nearest endpoint-line includes target.pt[1]
					dir = (target.pt[1] - ballPos).cross(relPos) > 0 ? MotionCmd::CCW : MotionCmd::CW;
				}
			}
			
			robot->pivot(ballPos, dir);
			break;
		}
			
		case State_Kick:
			robot->addText("Kick");
			robot->move(ballPos);
			robot->face(ballPos);
			robot->kick(255);
			break;
		
		case State_Done:
			robot->addText("Done");
			break;
	}
	
	return true;
}

void Gameplay::Behaviors::Kick::restart()
{
	_state = State_Approach1;
}

void Gameplay::Behaviors::Kick::setTargetGoal()
{
	setTarget(Geometry2d::Segment(
		Geometry2d::Point(Field_GoalWidth / 2, Field_Length),
		Geometry2d::Point(-Field_GoalWidth / 2, Field_Length)));
}

void Gameplay::Behaviors::Kick::setTarget(const Geometry2d::Segment& seg)
{
	_target = seg;
}
