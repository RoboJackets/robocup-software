#include "Bump.hpp"
#include <stdio.h>

using namespace Geometry2d;

namespace Gameplay
{
	namespace Behaviors
	{
		REGISTER_CONFIGURABLE(Bump)
	}
}

ConfigBool   *Gameplay::Behaviors::Bump::_face_ball;
ConfigDouble *Gameplay::Behaviors::Bump::_drive_around_dist;
ConfigDouble *Gameplay::Behaviors::Bump::_setup_to_charge_thresh;
ConfigDouble *Gameplay::Behaviors::Bump::_escape_charge_thresh;
ConfigDouble *Gameplay::Behaviors::Bump::_setup_ball_avoid;
ConfigDouble *Gameplay::Behaviors::Bump::_bump_complete_dist;

void Gameplay::Behaviors::Bump::createConfiguration(Configuration *cfg)
{
	_drive_around_dist = new ConfigDouble(cfg, "Bump/Drive Around Dist", 0.25);
	_face_ball = new ConfigBool(cfg, "Bump/Face Ball (otherwise target)", true);
	_setup_to_charge_thresh = new ConfigDouble(cfg, "Bump/Charge Thresh", 0.1);
	_escape_charge_thresh = new ConfigDouble(cfg, "Bump/Escape Charge Thresh", 0.1);
	_setup_ball_avoid = new ConfigDouble(cfg, "Bump/Setup Ball Avoid", Ball_Radius * 2.0);
	_bump_complete_dist = new ConfigDouble(cfg, "Bump/Bump Complete Distance", 0.5);
}

Gameplay::Behaviors::Bump::Bump(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	_state = State_Setup;
}

void Gameplay::Behaviors::Bump::restart()
{
	_state = State_Setup;
}

bool Gameplay::Behaviors::Bump::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	Line targetLine(ball().pos, target);
	
	// State changes
	if (_state == State_Setup)
	{
		if (targetLine.distTo(robot->pos) <= *_setup_to_charge_thresh && targetLine.delta().dot(robot->pos - ball().pos) <= -Robot_Radius)
		{
			_state = State_Charge;
		}
	} else if (_state == State_Charge)
	{
		if (Line(robot->pos, target).distTo(ball().pos) > *_escape_charge_thresh)
		{
			// Ball is in a bad place
			_state = State_Setup;
		}

		if (!robot->pos.nearPoint(ball().pos, *_bump_complete_dist))
		{
			_state = State_Done;
		}
	}
	
	// Driving
	if (_state == State_Setup)
	{
		// Move onto the line containing the ball and the_setup_ball_avoid target
		robot->addText(QString("%1").arg(targetLine.delta().dot(robot->pos - ball().pos)));
		if (targetLine.delta().dot(robot->pos - ball().pos) > -Robot_Radius)
		{
			// We're very close to or in front of the ball
			robot->addText("In front");
			robot->avoidBall(*_setup_ball_avoid);
			robot->move(ball().pos - targetLine.delta().normalized() * *_drive_around_dist);
		} else {
			// We're behind the ball
			robot->addText("Behind");
			robot->avoidBall(*_setup_ball_avoid);
			robot->move(targetLine.nearestPoint(robot->pos));
		}

		// DEBUG: use flag to change whether to face ball or into line
		if (*_face_ball)
		{
			robot->face(ball().pos);
		} else
		{
			robot->face(targetLine.nearestPoint(robot->pos)); // perpendicular to the line
		}

	} else if (_state == State_Charge)
	{
		robot->addText("Charge!");
		state()->drawLine(robot->pos, target, Qt::white);
		state()->drawLine(ball().pos, target, Qt::white);
		
		//We want to move in the direction of the target without path planning
		robot->worldVelocity(targetLine.delta() * 5.0); // Full speed
		robot->angularVelocity(0.0);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
