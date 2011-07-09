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
ConfigDouble *Gameplay::Behaviors::LineKick::_proj_time;
ConfigDouble *Gameplay::Behaviors::LineKick::_dampening;

void Gameplay::Behaviors::LineKick::createConfiguration(Configuration *cfg)
{
	_drive_around_dist = new ConfigDouble(cfg, "LineKick/Drive Around Dist", 0.25);
	_setup_to_charge_thresh = new ConfigDouble(cfg, "LineKick/Charge Thresh", 0.1);
	_escape_charge_thresh = new ConfigDouble(cfg, "LineKick/Escape Charge Thresh", 0.1);
	_setup_ball_avoid = new ConfigDouble(cfg, "LineKick/Setup Ball Avoid", Ball_Radius * 2.0);
	_accel_bias = new ConfigDouble(cfg, "LineKick/Accel Bias", 0.1);
	_facing_thresh = new ConfigDouble(cfg, "LineKick/Facing Thresh - Deg", 10);
	_max_speed = new ConfigDouble(cfg, "LineKick/Max Charge Speed", 1.5);
	_proj_time = new ConfigDouble(cfg, "LineKick/Ball Project Time", 0.4);
	_dampening = new ConfigDouble(cfg, "LineKick/Ball Project Dampening", 0.8);
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
	scaleAcc = 1.0;
	scaleSpeed = 1.0;
	scaleW = 1.0;
}

bool Gameplay::Behaviors::LineKick::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}

	// project the ball ahead to handle movement
	double dt = *_proj_time;
	Point ballPos = ball().pos + ball().vel * dt * _dampening->value();  // projecting
//	Point ballPos = ball().pos; // no projecting

	Line targetLine(ballPos, target);
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	double facing_thresh = cos(*_facing_thresh * DegreesToRadians);
	double facing_err = dir.dot((target - ballPos).normalized());
	
	// State changes
	if (_state == State_Setup)
	{
		if (targetLine.distTo(robot->pos) <= *_setup_to_charge_thresh &&
				targetLine.delta().dot(robot->pos - ballPos) <= -Robot_Radius &&
				facing_err >= facing_thresh &&
				robot->vel.mag() < 0.05)
		{
			_state = State_Charge;
		}
	} else if (_state == State_Charge)
	{
		if (Line(robot->pos, target).distTo(ballPos) > *_escape_charge_thresh)
		{
			// Ball is in a bad place
			_state = State_Setup;
		}
	}
	
	// Driving
	if (_state == State_Setup)
	{
		// Move onto the line containing the ball and the_setup_ball_avoid target
		robot->addText(QString("%1").arg(targetLine.delta().dot(robot->pos - ballPos)));
		Point moveGoal = ballPos - targetLine.delta().normalized() * (*_drive_around_dist + Robot_Radius);

		static const Segment left_field_edge(Point(-Field_Width / 2.0, 0.0), Point(-Field_Width / 2.0, Field_Length));
		static const Segment right_field_edge(Point(Field_Width / 2.0, 0.0), Point(Field_Width / 2.0, Field_Length));

		// Handle edge of field case
		float field_edge_thresh = 0.3;
		Segment behind_line(ballPos - targetLine.delta().normalized() * (*_drive_around_dist),
				ballPos - targetLine.delta().normalized() * 1.0);
		state()->drawLine(behind_line);
		Point intersection;
		if (left_field_edge.nearPoint(ballPos, field_edge_thresh) && behind_line.intersects(left_field_edge, &intersection))
		{
			moveGoal = intersection;
		} else if (right_field_edge.nearPoint(ballPos, field_edge_thresh) && behind_line.intersects(right_field_edge, &intersection))
		{
			moveGoal = intersection;
		}

		robot->addText("Setup");
		robot->avoidBall(*_setup_ball_avoid);
		robot->move(moveGoal);

		// face in a direction so that on impact, we aim at goal
		Point delta_facing = target - ballPos;
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
		state()->drawLine(ballPos, target, Qt::white);
		
		Point ballToTarget = (target - ballPos).normalized();
		Point robotToBall = (ballPos - robot->pos).normalized();
		Point driveDirection = robotToBall;

		// Drive directly into the ball
		double speed = min(robot->vel.mag() + (*_accel_bias * scaleAcc), _max_speed->value()); // enough of a bias to force it to accelerate
		robot->worldVelocity(driveDirection.normalized() * speed);

		// scale everything to adjust precision
		robot->setWScale(scaleW);
		robot->setVScale(scaleSpeed);

		robot->face(ballPos);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
