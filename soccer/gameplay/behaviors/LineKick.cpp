#include "LineKick.hpp"
#include <stdio.h>

using namespace Geometry2d;

Gameplay::Behaviors::LineKick::LineKick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	_state = State_Setup;
}

void Gameplay::Behaviors::LineKick::restart()
{
	_state = State_Setup;
}

bool Gameplay::Behaviors::LineKick::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	Line targetLine(ball().pos, target);
	
	// State changes
	if (_state == State_Setup)
	{
		if (targetLine.distTo(robot->pos) <= 0.1 && targetLine.delta().dot(robot->pos - ball().pos) <= -Robot_Radius)
		{
			_state = State_Charge;
		}
	} else if (_state == State_Charge)
	{
		if (Line(robot->pos, target).distTo(ball().pos) > 0.1)
		{
			// Ball is in a bad place
			_state = State_Setup;
		}
	}
	
	// Driving
	robot->face(ball().pos);
	if (_state == State_Setup)
	{
		// Move onto the line containing the ball and the target
		robot->addText(QString("%1").arg(targetLine.delta().dot(robot->pos - ball().pos)));
		if (targetLine.delta().dot(robot->pos - ball().pos) > -Robot_Radius)
		{
			// We're very close to or in front of the ball
			robot->addText("In front");
			robot->move(ball().pos - targetLine.delta().normalized() * 0.25);
		} else {
			// We're behind the ball
			robot->addText("Behind");
			robot->move(targetLine.nearestPoint(robot->pos));
		}
		robot->kick(0);
	} else if (_state == State_Charge)
	{
		robot->addText("Charge!");
		robot->kick(255);
		state()->drawLine(robot->pos, target, Qt::white);
		state()->drawLine(ball().pos, target, Qt::white);
		
		//FIXME - We want to move in the direction of the target without path planning
		robot->move(ball().pos + targetLine.delta() * 0.5);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
