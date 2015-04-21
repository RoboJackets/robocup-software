#include "StableLineKick.hpp"
#include <gameplay/tactics/passing/PassReceiver.hpp>
#include <math.h>

namespace Gameplay { REGISTER_CONFIGURABLE(StableLineKick) }

ConfigInt *Gameplay::StableLineKick::_kickPower;
ConfigDouble *Gameplay::StableLineKick::_backoffDistance;
ConfigDouble *Gameplay::StableLineKick::_maxExecutionError;
ConfigDouble *Gameplay::StableLineKick::_approachVelocity;

ConfigBool *Gameplay::StableLineKick::_debug;

void Gameplay::StableLineKick::createConfiguration(Configuration *cfg)
{
	_kickPower = new ConfigInt(cfg, "StableLineKick/Kick Power", 25);
	_backoffDistance = new ConfigDouble(cfg, "StableLineKick/Backoff Distance", 0.2);
	_maxExecutionError = new ConfigDouble(cfg, "StableLineKick/Max Execution Error", 0.015);
	_approachVelocity = new ConfigDouble(cfg, "StableLineKick/Approach Velocity", 1.0);

	_debug = new ConfigBool(cfg, "StablePivotKick/Debug", true);
}

Gameplay::StableLineKick::StableLineKick(GameplayModule *gameplay)
	: ActionBehavior(gameplay),
	  _state(Setup)
{
}

bool Gameplay::StableLineKick::isSettingUp()
{
	return _state == Setup;
}

bool Gameplay::StableLineKick::isSetup()
{
	return isAtPassPosition() && _state == Setup;
}

bool Gameplay::StableLineKick::isActing()
{
	return _state == Kick;
}

bool Gameplay::StableLineKick::isDone()
{
	return _state == Done;
}

void Gameplay::StableLineKick::restart()
{
	_state = Setup;
}

Geometry2d::Point Gameplay::StableLineKick::passPosition()
{
	using namespace Geometry2d;

	// Assume the ball is stationary for easier computations.
	Point ballPos = ball().pos;
	Point passVector = (actionTarget - ballPos).normalized();
	Point passPos = ballPos - passVector**_backoffDistance;

	return passPos;
}

float Gameplay::StableLineKick::passPositionError()
{
	using namespace Geometry2d;

	float errorPos = passPosition().distTo(robot->pos);
	Line passLine(robot->pos, robot->kickerBar().center());
	float errorAngle = passLine.distTo(actionTarget);
	return std::max(errorPos, errorAngle);
}

bool Gameplay::StableLineKick::isAtPassPosition()
{
	return passPositionError() < *_maxExecutionError;
}

float Gameplay::StableLineKick::passPathError()
{
	using namespace Geometry2d;

	float distError = Line(passPosition(), actionTarget).distTo(robot->pos);
	float distBall = (ball().pos - robot->pos).mag();
	return std::max(distError, distBall);
}

bool Gameplay::StableLineKick::isOnPassPath()
{
	return passPathError() < *_backoffDistance;
}

bool Gameplay::StableLineKick::run()
{
	using namespace Geometry2d;

	if(!robot || !robot->visible) {
		return false;
	}


	if ( partner ) {
		PassReceiver *rcvr = (PassReceiver *)partner;
		rcvr->kickPoint = passPosition();

		Point delta = ball().pos - robot->pos;
		rcvr->kickAngle = delta.angle();
	}


	// State changing
	if(_state == Setup) {
		if(isAtPassPosition()) {
			if(!partner || partner->isSetup() ) {
				_state = Kick;
			}
		}
	} else if(_state == Kick) {
		if(!isOnPassPath()) {
			_state = Setup;
		} else if(robot->hasBall()) {
			_state = Done;
		}
	} else if(_state == Done) {
		// Do nothing
	}

	// Debug visualization
	if(*_debug) {
		if(_state == Setup) {
			robot->addText(QString("SLK: Setup"), Qt::yellow, QString("StableLineKick"));
			robot->addText(QString("SLK error: %1").arg(passPositionError()), Qt::yellow, QString("StableLineKick"));
			// Draw circle at pass position
			state()->drawCircle(passPosition(), Robot_Radius + 0.05, Qt::yellow, QString("StableLineKick"));
			// Draw line to show where robot is going to kick
			float targetDist = (actionTarget - ball().pos).mag() + Robot_Radius;
			Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
			state()->drawLine(robot->pos, robot->pos + faceVector*targetDist, Qt::yellow, QString("StableLineKick"));
		} else if(_state == Kick) {
			robot->addText(QString("SLK: Kick"), Qt::green, QString("StableLineKick"));
			robot->addText(QString("SLK error: %1").arg(passPathError()), Qt::green, QString("StableLineKick"));
			// Draw line to show where robot is going to kick
			float targetDist = (actionTarget - ball().pos).mag() + Robot_Radius;
			Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
			state()->drawLine(robot->pos, robot->pos + faceVector*targetDist, Qt::green, QString("StableLineKick"));
		} else if(_state == Done) {
			// Do nothing
			robot->addText(QString("SLK: Done"), Qt::red, QString("StableLineKick"));
		}
	}

	// Commands
	if(_state == Setup) {
		robot->resetAvoidBall();
		robot->move(passPosition(), false);
		robot->face(ball().pos);
	} else if(_state == Kick) {
		robot->kick(*_kickPower);
		// FIXME: This has the side effect of always being on, even after play termination
		// because it modifies a OurRobot internal variable.
		robot->disableAvoidBall();
		// Move towards receive target to kick the ball on that path.
		//robot->move(actionTarget, false);
		Point driveVector = (ball().pos - robot->pos).normalized();
		robot->worldVelocity(driveVector**_approachVelocity);
		robot->face(ball().pos);

		if ( partner ) {
			PassReceiver *rcvr = (PassReceiver *)partner;
			rcvr->partnerDidKick();
		}
	}


	return true;
}
