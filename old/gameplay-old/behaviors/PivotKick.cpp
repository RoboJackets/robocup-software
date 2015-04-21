//Eric is sleeping
#include "PivotKick.hpp"

#include <Utils.hpp>
#include <gameplay/evaluation/WindowEvaluator.hpp>

using namespace std;
using namespace Geometry2d;

namespace Gameplay
{
	namespace Behaviors
	{
		REGISTER_CONFIGURABLE(PivotKick)
	}
}

ConfigDouble *Gameplay::Behaviors::PivotKick::_aim_Speed;
ConfigDouble *Gameplay::Behaviors::PivotKick::_kick_Completed_Threshold;
ConfigDouble *Gameplay::Behaviors::PivotKick::_initial_Accuracy;
ConfigDouble *Gameplay::Behaviors::PivotKick::_accuracy_Delta;
ConfigDouble *Gameplay::Behaviors::PivotKick::_fireNowThreshold;

ConfigBool *Gameplay::Behaviors::PivotKick::_land_on_target;
ConfigBool *Gameplay::Behaviors::PivotKick::_allow_chipping;


void Gameplay::Behaviors::PivotKick::createConfiguration(Configuration* cfg)
{
	_aim_Speed = new ConfigDouble(cfg, "PivotKick/Aim Speed", 0.5 * M_PI);
	_kick_Completed_Threshold = new ConfigDouble(cfg, "PivotKick/Kick Completed Threshold", 0.5);
	_initial_Accuracy = new ConfigDouble(cfg, "PivotKick/Initial Accuracy", cos(10 * DegreesToRadians));
	_accuracy_Delta  = new ConfigDouble(cfg, "PivotKick/Accuracy Delta", 0.000);
	_fireNowThreshold  = new ConfigDouble(cfg, "PivotKick/Fire Now Threshold", cos(3 * DegreesToRadians));

    _land_on_target = new ConfigBool(cfg, "PivotKick/Land On Target", true);
    _allow_chipping = new ConfigBool(cfg, "PivotKick/Allow Chipping", true);
}

Gameplay::Behaviors::PivotKick::PivotKick(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay), _capture(gameplay), chip_calib(gameplay)
{
	restart();
	
	target.pt[0] = Point(Field_GoalWidth / 2, Field_Length);
	target.pt[1] = Point(-Field_GoalWidth / 2, Field_Length);
	_capture.target = target.pt[0];

	use_windowing = true; // Defaults to windowing on.
}

void Gameplay::Behaviors::PivotKick::restart()
{
	_state = State_Capture;
	_kicked = false;
	_ccw = true;
	_capture.restart();
	_capture.target = target.pt[0];
	enable_kick = true;
	enable_desparate_kick = true;
	dribble_speed = 50;
	use_chipper = false;
	kick_power = 255;
}

bool Gameplay::Behaviors::PivotKick::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	// force sub-behavior to have its robot assigned
	_capture.robot = robot;

	// The direction we're facing
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	
	uint64_t now = timestamp();
	
	//	track kick time
	if ( _state != State_Aim ) {
		_lastKickTime = robot->lastKickTime();
	}

	// State changes
	Point toBall = (ball().pos - robot->pos).normalized();
	if (_state == State_Capture)
	{
		kick_ready = false;
		if (_capture.done())
		{
			_state = State_Aim;
			_lastError = 0;
			_lastDelta = 0;
			_ccw = ((target.pt[0] - ball().pos).cross(target.pt[1] - ball().pos) > 0);
		}
		_accuracy = *_initial_Accuracy;
	} else if (_state == State_Aim)
	{
		// _lastBallTime is the last time we had the ball
		if (robot->hasBall())
		{
			_lastBallTime = now;
		}
		
		// watch for ball leaving the robot
		

		if ((!robot->hasBall() && (state()->timestamp - _lastBallTime) > 500000) || !ball().pos.nearPoint(robot->pos, *_kick_Completed_Threshold))
            //if ( robot->lastKickTime() > _lastKickTime )
		{
			if (_kicked)
			{
				_state = State_Done;
			} else {
				_state = State_Capture;
				_capture.restart();
			}
		}
	}
	
	state()->drawLine(ball().pos, target.pt[0], Qt::red);
	state()->drawLine(ball().pos, target.pt[1], Qt::black);
	state()->drawLine(target, Qt::yellow);
	
	// Driving
	if (_state == State_Capture)
	{
		robot->addText("Capturing");
		_capture.target = target.pt[0];
		_capture.run();
		robot->kick(0); // "Un-kick" to prevent accidental kicks
	}  else if (_state == State_Aim)
	{
		Segment windowed_target = target;
		if(use_windowing)
		{
			robot->addText("Evaling", QColor(255,255,255), "PivotKick");
			WindowEvaluator we(state());
			we.debug = true;
			we.exclude.clear();
			we.exclude.push_back(robot->pos);
			we.run(ball().pos, target);
			if(we.windows.size() > 0 && we.best()->segment.length() > Ball_Diameter)
			{
				windowed_target = we.best()->segment;
			}
		}

		state()->drawLine(robot->pos, robot->pos + dir * 8, Qt::white);
		state()->drawLine(ball().pos, windowed_target.center(), Qt::yellow);
		state()->drawLine(robot->pos, robot->pos + (ball().pos - robot->pos).normalized() * 8, Qt::green);
		
		// See if it's time to kick
		float error = dir.dot((windowed_target.center() - ball().pos).normalized());
		float delta = error - _lastError;
		
		if ((error >= *_fireNowThreshold || (error >= _accuracy && _lastDelta > 0 && delta <= 0)))
		{
			if(enable_kick)
			{
				robot->immediate(false);
				if (robot->chipper_available() && use_chipper && (*_allow_chipping))
				{
					if(*_land_on_target)
						robot->chip(chip_calib.chipPowerForDistance(windowed_target.center().distTo(ball().pos)));
					else
						robot->chip(kick_power);
					robot->addText("CHIP");
				} else
				{
					robot->kick(kick_power);
					robot->addText("KICK");
				}
				_kicked = true;
			}
			kick_ready = true;
		} else {
			robot->addText("Aim");
			kick_ready = false;
		}
		
		_lastError = error;
		_lastDelta = delta;
		
		// Decide which direction to rotate around the ball
		Point rb = ball().pos - robot->pos;
		if (rb.cross(windowed_target.center() - ball().pos) > 0)
		{
			_ccw = true;
		} else if ((windowed_target.center() - ball().pos).cross(rb) > 0)
		{
			_ccw = false;
		}
		robot->addText(QString("Aim %1 %2 %3 %4").arg(
			QString::number(acos(error) * RadiansToDegrees),
			QString::number(delta),
			QString::number(_accuracy),
			QString::number(_ccw ? 1 : 0)));
		
		_accuracy -= *_accuracy_Delta;
		_accuracy = max(0.0f, _accuracy);

		robot->face(ball().pos);
		if(use_chipper && _land_on_target)
			robot->dribble(chip_calib.dribble_speed());
		else
			robot->dribble(dribble_speed);

	} else {
		robot->addText("Done");
		if (robot->chipper_available() && use_chipper && (*_allow_chipping))
			robot->chip(0);
		else
			robot->kick(0);
		return false;
	}
	
	return true;

}
