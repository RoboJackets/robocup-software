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
ConfigDouble *Gameplay::Behaviors::Bump::_accel_bias;
ConfigDouble *Gameplay::Behaviors::Bump::_facing_thresh;

void Gameplay::Behaviors::Bump::createConfiguration(Configuration *cfg)
{
	_drive_around_dist = new ConfigDouble(cfg, "Bump/Drive Around Dist", 0.45);
	_face_ball = new ConfigBool(cfg, "Bump/Face Ball otherwise target", true);
	_setup_to_charge_thresh = new ConfigDouble(cfg, "Bump/Charge Thresh", 0.1);
	_escape_charge_thresh = new ConfigDouble(cfg, "Bump/Escape Charge Thresh", 0.1);
	_setup_ball_avoid = new ConfigDouble(cfg, "Bump/Setup Ball Avoid", 1.0);
	_bump_complete_dist = new ConfigDouble(cfg, "Bump/Bump Complete Distance", 0.5);
	_accel_bias = new ConfigDouble(cfg, "Bump/Accel Bias", 0.1);
	_facing_thresh = new ConfigDouble(cfg, "Bump/Facing Thresh - Deg", 10);
}

Gameplay::Behaviors::Bump::Bump(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	_state = State_Setup;
	target = Point(0.0, Field_Length);
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
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	double facing_thresh = cos(*_facing_thresh * DegreesToRadians);
	double facing_err = dir.dot((target - ball().pos).normalized());
//	robot->addText(QString("Err:%1,T:%2").arg(facing_err).arg(facing_thresh));

	// State changes
	if (_state == State_Setup)
	{
		if (targetLine.distTo(robot->pos) <= *_setup_to_charge_thresh &&
				targetLine.delta().dot(robot->pos - ball().pos) <= -Robot_Radius &&
				facing_err >= facing_thresh)
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

		// FIXME: should finish at some point, but this condition is bad
//		if (!robot->pos.nearPoint(ball().pos, *_bump_complete_dist))
//		{
//			_state = State_Done;
//		}
	}
	
	// Driving
	if (_state == State_Setup)
	{
		// Move onto the line containing the ball and the_setup_ball_avoid target
		robot->addText(QString("%1").arg(targetLine.delta().dot(robot->pos - ball().pos)));
		Segment behind_line(ball().pos - targetLine.delta().normalized() * (*_drive_around_dist + Robot_Radius),
				ball().pos - targetLine.delta().normalized() * 5.0);
		if (targetLine.delta().dot(robot->pos - ball().pos) > -Robot_Radius)
		{
			// We're very close to or in front of the ball
			robot->addText("In front");
			robot->avoidBallRadius(*_setup_ball_avoid);
			robot->move(ball().pos - targetLine.delta().normalized() * (*_drive_around_dist + Robot_Radius));
		} else {
			// We're behind the ball
			robot->addText("Behind");
			robot->avoidBallRadius(*_setup_ball_avoid);
			robot->move(behind_line.nearestPoint(robot->pos));
			state()->drawLine(behind_line);
		}

		// face in a direction so that on impact, we aim at goal
		Point delta_facing = target - ball().pos;
		robot->face(robot->pos + delta_facing);

	} else if (_state == State_Charge)
	{
		robot->addText("Charge!");
		state()->drawLine(robot->pos, target, Qt::white);
		state()->drawLine(ball().pos, target, Qt::white);

		Point ballToTarget = (target - ball().pos).normalized();
//		Point robotToBall = (ball().pos - robot->pos).normalized();
		Point driveDirection = (ball().pos - ballToTarget * Robot_Radius) - robot->pos;
		
		//We want to move in the direction of the target without path planning
		double speed =  robot->vel.mag() + *_accel_bias; // enough of a bias to force it to accelerate
		robot->worldVelocity(driveDirection.normalized() * speed);
		robot->face(ball().pos);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
