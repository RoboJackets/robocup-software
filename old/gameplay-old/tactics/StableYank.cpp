/*
 * StableYank.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: Matthew Barulic
 */

#include "StableYank.hpp"

namespace Gameplay {

REGISTER_CONFIGURABLE(StableYank)

ConfigInt *StableYank::_dribblerPower;
ConfigDouble *StableYank::_yankDistance;
ConfigDouble *StableYank::_aimError;
ConfigDouble *StableYank::_moveError;

ConfigBool *StableYank::_debug;

void StableYank::createConfiguration(Configuration *cfg)
{
	_dribblerPower = new ConfigInt(cfg, "StableYank/Dribbler Power", 150);
	_yankDistance = new ConfigDouble(cfg, "StableYank/Yank Distance", 0.5);
	_aimError = new ConfigDouble(cfg, "StableYank/Aim Error", 0.01);
	_moveError = new ConfigDouble(cfg, "StableYank/Move Error", 0.2);

	_debug = new ConfigBool(cfg, "StableYank/Debug", true);
}

StableYank::StableYank(GameplayModule *gameplay)
	: ActionBehavior(gameplay),
	  _state(Capture)
{
	_capture = new Behaviors::Capture(gameplay);
}

bool StableYank::isSettingUp()
{
	return _state == Capture || _state == Aim;
}

bool StableYank::isSetup()
{
	return _capture->done() && robot->hasBall() && _state == Aim;
}

bool StableYank::isActing()
{
	return _state == Yank;
}

bool StableYank::isDone()
{
	return _state == Done;
}

void StableYank::restart()
{
	_state = Capture;
}

Geometry2d::Point StableYank::getCaptureTarget()
{
	using namespace Geometry2d;

	Point captureTarget = ball().pos + (ball().pos - actionTarget).normalized();

	return captureTarget;
}

float StableYank::faceDistFromCaptureTarget()
{
	using namespace Geometry2d;

	Point captureTarget = getCaptureTarget();

	Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
	Line botToBall(robot->pos, robot->pos + faceVector);
	float dist = botToBall.distTo(captureTarget);

	return dist;
}

bool StableYank::isFacingCaptureTarget()
{
	 return faceDistFromCaptureTarget() < *_aimError;
}

Geometry2d::Point StableYank::getPhase1Target()
{
	using namespace Geometry2d;

	Point dirVector = actionTarget - robot->pos;

	dirVector = dirVector.normalized();

	dirVector.rotate(Point(0,0), 45);

	dirVector *= *_yankDistance;

	Point target = _phase1StartPos + dirVector;

	//TODO Check if this location is actually free. If not, pick the location on the other side.

	return target;
}

bool StableYank::run()
{
	using namespace Geometry2d;

	// Make sure I have a valid robot.
	if(!robot || !robot->visible) {
		return false;
	}

	// State changes
	if(_state == Capture) {
		if(_capture->done())
		{
			_state = Aim;
		}
	} else if(_state == Aim) {
		if(isFacingCaptureTarget() && robot->hasBall())
		{
			_state = Yank;
			_phase1StartPos = robot->pos;
		} else if(!robot->hasBall())
		{
			_capture->restart();
			_state = Capture;
		}
	} else if(_state == Yank) {
		if(robot->pos.distTo(getPhase1Target()) < *_moveError) {
			_state = Done;
		}
	}

	// Commands
	if(_state == Capture) {
		_capture->robot = this->robot;
		_capture->target = getCaptureTarget();
		_capture->run();
	} else if(_state == Aim) {
		robot->face(getCaptureTarget());
		robot->dribble(*_dribblerPower);
	} else if(_state == Yank) {
		// Send yank commands
		robot->face(_phase1StartPos);
		robot->dribble(*_dribblerPower);
		robot->move(getPhase1Target(), false);
	}

	// Debug stuff
	if(*_debug) {
		state()->drawCircle(actionTarget, 0.1f, Qt::blue, QString("StableYank"));
		if(_state == Capture) {
			robot->addText(QString("SY: Capture"), Qt::white, QString("StableYank"));
		} else if(_state == Aim) {
			robot->addText(QString("SY: Aim"), Qt::white, QString("StableYank"));
			state()->drawCircle(getCaptureTarget(), 0.01f, Qt::red, QString("StableYank"));
			Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
			state()->drawLine(robot->pos, robot->pos + faceVector, Qt::yellow, QString("StableYank"));
			robot->addText(QString("Error: %1").arg(faceDistFromCaptureTarget()), Qt::white, QString("StableYank"));
		} else if(_state == Yank) {
			robot->addText(QString("SY: Yank"), Qt::white, QString("StableYank"));
			robot->addText(QString("Error: %1").arg(robot->pos.distTo(getPhase1Target())), Qt::white, QString("StableYank"));
			state()->drawCircle(getPhase1Target(), 0.01f, Qt::red, QString("StableYank"));
		} else if(_state == Done) {
			robot->addText(QString("SY: Done"), Qt::green, QString("StableYank"));
		}
	}

	return true;
}

} /* namespace Gameplay */
