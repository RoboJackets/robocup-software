#include "LineKick.hpp"
#include <stdio.h>

using namespace std;
using namespace Geometry2d;

namespace Gameplay
{
	namespace Behaviors
	{
		REGISTER_CONFIGURABLE(LineKick)
	}
}

ConfigDouble *Gameplay::Behaviors::LineKick::_drive_around_dist;
ConfigDouble *Gameplay::Behaviors::LineKick::_setup_to_charge_thresh;
ConfigDouble *Gameplay::Behaviors::LineKick::_escape_charge_thresh;
ConfigDouble *Gameplay::Behaviors::LineKick::_setup_ball_avoid;
ConfigDouble *Gameplay::Behaviors::LineKick::_accel_bias;
ConfigDouble *Gameplay::Behaviors::LineKick::_facing_thresh;
ConfigDouble *Gameplay::Behaviors::LineKick::_max_speed;

void Gameplay::Behaviors::LineKick::createConfiguration(Configuration *cfg)
{
	_drive_around_dist = new ConfigDouble(cfg, "LineKick/Drive Around Dist", 0.25);
	_setup_to_charge_thresh = new ConfigDouble(cfg, "LineKick/Charge Thresh", 0.1);
	_escape_charge_thresh = new ConfigDouble(cfg, "LineKick/Escape Charge Thresh", 0.1);
	_setup_ball_avoid = new ConfigDouble(cfg, "LineKick/Setup Ball Avoid", Ball_Radius * 2.0);
	_accel_bias = new ConfigDouble(cfg, "LineKick/Accel Bias", 0.1);
	_facing_thresh = new ConfigDouble(cfg, "LineKick/Facing Thresh - Deg", 10);
	_max_speed = new ConfigDouble(cfg, "LineKick/Max Charge Speed", 1.5);
}

Gameplay::Behaviors::LineKick::LineKick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	restart();
	target = Geometry2d::Point(0.0, Field_Length);
}

void Gameplay::Behaviors::LineKick::restart()
{
	_state = State_Setup;
	use_chipper = false;
	kick_power = 255;
}

bool Gameplay::Behaviors::LineKick::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}

	Line targetLine(ball().pos, target);
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	double facing_thresh = cos(*_facing_thresh * DegreesToRadians);
	double facing_err = dir.dot((target - ball().pos).normalized());
	
	// State changes
	if (_state == State_Setup)
	{
		if (targetLine.distTo(robot->pos) <= *_setup_to_charge_thresh &&
				targetLine.delta().dot(robot->pos - ball().pos) <= -Robot_Radius &&
				facing_err >= facing_thresh &&
				robot->vel.mag() < 0.05)
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
	}
	
	// Driving
	if (_state == State_Setup)
	{
		// Move onto the line containing the ball and the_setup_ball_avoid target
		robot->addText(QString("%1").arg(targetLine.delta().dot(robot->pos - ball().pos)));
		Segment behind_line(ball().pos - targetLine.delta().normalized() * (*_drive_around_dist + Robot_Radius),
				ball().pos - targetLine.delta().normalized() * 1.0);
		if (targetLine.delta().dot(robot->pos - ball().pos) > -Robot_Radius)
		{
			// We're very close to or in front of the ball
			robot->addText("In front");
			robot->avoidBall(*_setup_ball_avoid);
			robot->move(ball().pos - targetLine.delta().normalized() * (*_drive_around_dist + Robot_Radius));
		} else
		{
			// We're behind the ball
			robot->addText("Behind");
			robot->avoidBall(*_setup_ball_avoid);
			robot->move(behind_line.nearestPoint(robot->pos));
			state()->drawLine(behind_line);
		}

		// face in a direction so that on impact, we aim at goal
		Point delta_facing = target - ball().pos;
		robot->face(robot->pos + delta_facing);

		robot->kick(0);
	} else if (_state == State_Charge)
	{
		robot->addText("Charge!");
		if (use_chipper)
		{
			robot->chip(kick_power);
		} else
		{
			robot->kick(kick_power);
		}
		state()->drawLine(robot->pos, target, Qt::white);
		state()->drawLine(ball().pos, target, Qt::white);
		
		Point ballToTarget = (target - ball().pos).normalized();
		Point robotToBall = (ball().pos - robot->pos).normalized();
		Point driveDirection = robotToBall;
//		Point driveDirection = (ball().pos - ballToTarget * Robot_Radius) - robot->pos; // original attempt

		//We want to move in the direction of the target without path planning
		double speed = min(robot->vel.mag() + *_accel_bias, _max_speed->value()); // enough of a bias to force it to accelerate
		robot->worldVelocity(driveDirection.normalized() * speed);
//		robot->angularVelocity(0.0);
		robot->face(ball().pos);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
