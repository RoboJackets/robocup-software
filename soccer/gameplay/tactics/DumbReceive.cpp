#include "DumbReceive.hpp"

namespace Gameplay { REGISTER_CONFIGURABLE(DumbReceive) }

ConfigDouble *Gameplay::DumbReceive::_backoffDistance;
ConfigDouble *Gameplay::DumbReceive::_maxExecutionError;
ConfigDouble *Gameplay::DumbReceive::_kickTimeout;

ConfigBool *Gameplay::DumbReceive::_debug;

void Gameplay::DumbReceive::createConfiguration(Configuration *cfg)
{
	_backoffDistance = new ConfigDouble(cfg, "DumbReceive/Backoff Distance", 0.2);
	_maxExecutionError = new ConfigDouble(cfg, "DumbReceive/Max Execution Error", 0.01);
	_kickTimeout = new ConfigDouble(cfg, "DumbReceive/Kick Timeout", 2.0);

	_debug = new ConfigBool(cfg, "StablePivotKick/Debug", true);
}

Gameplay::DumbReceive::DumbReceive(GameplayModule *gameplay)
	: PassingBehavior(gameplay),
	  _state(Setup),
	  _success(false)
{
}

bool Gameplay::DumbReceive::done()
{
	return _state == Done;
}

void Gameplay::DumbReceive::restart()
{
	_state = Setup;
	_success = false;
}

bool Gameplay::DumbReceive::success()
{
	return _success;
}

bool Gameplay::DumbReceive::ready()
{
	using namespace Geometry2d;

	float dist = (receiveTarget - robot->pos).mag();

	return dist < *_maxExecutionError;
}

Geometry2d::Point Gameplay::DumbReceive::receivePosition()
{
	using namespace Geometry2d;

	// Assume the ball is stationary for easier computations.
	Point ballPos = ball().pos;
	Point passVector = (receiveTarget - ballPos).normalized();
	Point passPos = ballPos - passVector**_backoffDistance;

	return passPos;
}

bool Gameplay::DumbReceive::run()
{
	using namespace Geometry2d;

	if(!robot || !robot->visible) {
		return false;
	}

	// State changing
	if(_state == Setup) {
		if(isAtPassPosition()) {
			_state = Receive;
		}
	} else if(_state == Receive) {
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
			robot->addText(QString("SLK: Setup"), Qt::yellow, QString("DumbReceive"));
			// Draw circle at pass position
			state()->drawCircle(receivePosition(), Robot_Radius + 0.05, Qt::yellow, QString("DumbReceive"));
			// Draw line to show where robot is going to kick
			float targetDist = (receiveTarget - ball().pos).mag() + Robot_Radius;
			Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
			state()->drawLine(robot->pos, robot->pos + faceVector*targetDist, Qt::yellow, QString("DumbReceive"));
		} else if(_state == Receive) {
			robot->addText(QString("SLK: Kick"), Qt::green, QString("DumbReceive"));
			// Draw line to show where robot is going to kick
			float targetDist = (receiveTarget - ball().pos).mag() + Robot_Radius;
			Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
			state()->drawLine(robot->pos, robot->pos + faceVector*targetDist, Qt::green, QString("DumbReceive"));
		} else if(_state == Done) {
			// Do nothing
			robot->addText(QString("SLK: Done"), Qt::red, QString("DumbReceive"));
		}
	}

	// Commands
	if(_state == Setup) {
		robot->avoidBall();
		robot->move(receivePosition(), false);
		robot->face(ball().pos);
	} else if(_state == Receive) {
		robot->kick(*_kickPower);
		// FIXME: This has the side effect of always being on, even after play termination
		// because it modifies a OurRobot internal variable.
		robot->disableAvoidBall();
		// Move towards receive target to kick the ball on that path.
		//robot->move(receiveTarget, false);
		Point driveVector = (ball().pos - robot->pos).normalized();
		robot->worldVelocity(driveVector**_kickTimeout);
		robot->face(ball().pos);
	}


	return true;
}
