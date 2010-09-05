#include "Kick.hpp"

#include <stdio.h>
#include <algorithm>

using namespace std;

Gameplay::Behaviors::Kick::Kick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	_target.pt[0] = Geometry2d::Point(-Field_GoalWidth / 2, 0);
	_target.pt[1] = Geometry2d::Point(Field_GoalWidth / 2, 0);
	
	restart();
}

bool Gameplay::Behaviors::Kick::run()
{
	Geometry2d::Segment target = _target;
	if ((robot->pos - target.pt[0]).magsq() < (robot->pos - target.pt[1]).magsq())
	{
// 		swap(target.pt[0], target.pt[1]);
	}
	
	Geometry2d::Point ballPos = ball().pos;
	
	switch (_state)
	{
		case State_Approach1:
			robot->addText("Approach1");
			robot->move(ballPos + (ballPos - target.pt[0]).normalized() * (Robot_Radius + Ball_Radius));
			robot->face(target.pt[0]);
			
			if (robot->pos.nearPoint(ballPos, Robot_Radius + Ball_Radius + 0.03))
			{
				_state = State_Approach2;
			}
			break;
			
		case State_Approach2:
			robot->addText("Approach2");
			robot->move(ballPos);
			robot->face(ballPos);
			robot->dribble(255);
			
			if (robot->hasBall)
			{
				_state = State_Aim;
				_lastError = INFINITY;
			}
			break;
			
		case State_Aim:
		{
			float z = (target.pt[0] - ballPos).cross(target.pt[1] - ballPos);
			robot->pivot(ballPos, z > 0 ? MotionCmd::CCW : MotionCmd::CW);
			
			Geometry2d::Point rd = Geometry2d::Point::direction(robot->angle * DegreesToRadians);
			state()->drawLine(robot->pos, robot->pos + rd);
			state()->drawLine(robot->pos, target.center());
			float error = rd.dot((target.center() - robot->pos).normalized());
			robot->addText(QString("Aim %1").arg(error));
			
			if (!isinf(_lastError))
			{
				if (error < _lastError && error > 0.999)
				{
					// Past the best position
					_state = State_Kick;
				}
			}
			_lastError = error;
			
			break;
		}
			
		case State_Kick:
			robot->addText("Kick");
			robot->move(ballPos);
			robot->face(ballPos);
			robot->kick(255);
			if (!robot->pos.nearPoint(ballPos, 0.1))
			{
				_state = State_Done;
			}
			break;
		
		case State_Done:
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
}
