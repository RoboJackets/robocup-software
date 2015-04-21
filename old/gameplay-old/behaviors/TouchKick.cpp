#include "TouchKick.hpp"
#include <stdio.h>

using namespace std;
using namespace Geometry2d;

namespace Gameplay {
	namespace Behaviors {
		REGISTER_CONFIGURABLE(TouchKick)
	}
}

ConfigDouble *Gameplay::Behaviors::TouchKick::_done_thresh;
ConfigDouble *Gameplay::Behaviors::TouchKick::_rotate_thresh;
ConfigDouble *Gameplay::Behaviors::TouchKick::_proj_time;
ConfigDouble *Gameplay::Behaviors::TouchKick::_dampening;

void Gameplay::Behaviors::TouchKick::createConfiguration(Configuration *cfg)
{
	_done_thresh = new ConfigDouble(cfg, "Touch/Done State Thresh", 0.11);
	_rotate_thresh = new ConfigDouble(cfg, "TouchKick/Rotate Thresh", 0.1);
	_proj_time = new ConfigDouble(cfg, "TouchKick/Ball Project Time", 0.4);
	_dampening = new ConfigDouble(cfg, "TouchKick/Ball Project Dampening", 0.8);
}

Gameplay::Behaviors::TouchKick::TouchKick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay),
    ballClose(false)
{
	restart();
	target = Geometry2d::Point(0.0, Field_Length);
	enable_kick = true;
}

void Gameplay::Behaviors::TouchKick::restart()
{
	_state = State_Setup;
	use_chipper = false;
	kick_power = 255;
	ballClose = false;
}

bool Gameplay::Behaviors::TouchKick::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	double dt = *_proj_time;
	Point ballPos = ball().pos + ball().vel * dt * _dampening->value();

	if(ballPos.distTo(robot->pos) <= *_done_thresh)
	{
		ballClose = true;
	}

	// State changes
	if (_state == State_Setup)
	{
		//if the ball if further away than the back off distance for the setup stage
		if(ballClose && ballPos.distTo(robot->pos) > *_done_thresh)
		{
			_state = State_Done;
		}
		if(targetRot - robot->angle <= *_rotate_thresh)
		{
			_state = State_Ready;
		}

	} else if (_state == State_Ready)
	{
		if(targetRot - robot->angle >= *_rotate_thresh)
		{
			_state = State_Setup;
		}
		if(ballClose && ballPos.distTo(robot->pos) > *_done_thresh)
		{
			_state = State_Done;
		}
	}


	// Driving
	if (_state == State_Setup)
	{
		robot->addText("Setup");
		// Geometry2d::Point robotPos = robot->kickerBar().center();
		Geometry2d::Point robotToBall;
		Geometry2d::Point robotToTarget;
		Geometry2d::Point targetPoint;
		robotToBall = (ballPos - robot->pos).normalized();
		robotToTarget = (target - robot->pos).normalized();
		targetPoint = robot->pos + (robotToBall + robotToTarget) / 2.f;
		targetRot = targetPoint.angle();
		robot->face(targetPoint);

		Geometry2d::Segment targetLine(targetPoint, robot->pos);
		state()->drawLine(robot->pos, targetPoint, Qt::white);

	} else if (_state == State_Ready)
	{
		robot->addText("Ready");
		if (use_chipper)
		{
			robot->chip(kick_power);
		} else
		{
			robot->kick(kick_power);
		}

	} else {
		robot->addText("Done");
		return false;
	}
	return true;
}
