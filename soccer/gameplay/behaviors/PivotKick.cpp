#include "PivotKick.hpp"

using namespace Geometry2d;

Gameplay::Behaviors::PivotKick::PivotKick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	_state = State_Approach;
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
	
	// State changes
	if (_state == State_Approach)
	{
		if (robot->hasBall)
		{
			_state = State_Aim;
		}
	} else if (_state == State_Aim)
	{
		if (!robot->hasBall)
		{
			_state = State_Approach;
		}
	}
	
	// Driving
	robot->face(ball().pos);
	if (_state == State_Approach)
	{
		robot->addText("Approach");
		robot->kick(0);
		robot->move(ball().pos - (target.pt[0] - ball().pos).normalized() * Robot_Radius);
		robot->face(ball().pos);
	} else if (_state == State_Aim)
	{
		robot->addText("Aim");
		state()->drawLine(robot->pos, robot->pos + (ball().pos - robot->pos).normalized() * 8, Qt::white);
		state()->drawLine(ball().pos, target.center(), Qt::white);
		
		robot->kick(0);
		robot->move(Point::rotate(robot->pos, ball().pos, 3));
		robot->face(ball().pos);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
