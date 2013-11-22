#include "DumbReceive.hpp"
#include "Utils.hpp"
#include <math.h>

namespace Gameplay { REGISTER_CONFIGURABLE(DumbReceive) }

ConfigDouble *Gameplay::DumbReceive::_backoffDistance;
ConfigDouble *Gameplay::DumbReceive::_maxExecutionError;
ConfigDouble *Gameplay::DumbReceive::_kickTimeout;

ConfigBool *Gameplay::DumbReceive::_debug;

void Gameplay::DumbReceive::createConfiguration(Configuration *cfg)
{
	_backoffDistance = new ConfigDouble(cfg, "DumbReceive/Backoff Distance", 0.078674155);
	_maxExecutionError = new ConfigDouble(cfg, "DumbReceive/Max Execution Error", 0.015);
	_kickTimeout = new ConfigDouble(cfg, "DumbReceive/Kick Timeout", 3.0);

	_debug = new ConfigBool(cfg, "StablePivotKick/Debug", true);
}

Gameplay::DumbReceive::DumbReceive(GameplayModule *gameplay)
	: ActionBehavior(gameplay),
	  _state(Setup),
	  _success(false),
	  _kickDelta(0),
	  timeout(4)
{
}

bool Gameplay::DumbReceive::isSettingUp()
{
	return _state == Setup;
}

bool Gameplay::DumbReceive::isSetup()
{
	return isAtReceivePosition() && _state == Setup;
}

bool Gameplay::DumbReceive::isActing()
{
	return _state == Receive_PassKicking || _state == Receive_PassDone;
}

bool Gameplay::DumbReceive::isDone()
{
	return _state == Done;
}

void Gameplay::DumbReceive::restart()
{
	_state = Setup;
	_success = false;
	_kickDelta = 0;
}

Geometry2d::Point Gameplay::DumbReceive::receivePosition()
{
	using namespace Geometry2d;

	Point receiveTarget;  // Point at which to place robot mouth
	Point passVector;     // Direction of pass
	if(_state == Setup) {
		receiveTarget = actionTarget;
		passVector = actionTarget - ball().pos;
	} else if(_state == Receive_PassKicking && partner) {
		OurRobot* passer = partner->robot;
		// The direction passer faces is the passing direction
		passVector = passer->kickerBar().center() - passer->pos;
		Line passLine(ball().pos, ball().pos + passVector);
		// Put mouth on the line
		// Using robot->pos would have caused the receiver to move backwards
		receiveTarget = passLine.nearestPoint(robot->kickerBar().center());
	} else if(_state == Receive_PassDone) {
		Line passLine(ball().pos, ball().pos + ball().vel);
		receiveTarget = passLine.nearestPoint(robot->kickerBar().center());
		passVector = ball().vel;
	} else if(_state == Done) {
		// Return start, if nowhere else
		receiveTarget = actionTarget;
		// passVector is (0,0)
	}

	// Calculate location of robot center
	receiveTarget += passVector **_backoffDistance;

	return receiveTarget;
}

float Gameplay::DumbReceive::receivePositionError()
{
	if ( !robot ) return 100;
	
	using namespace Geometry2d;
	float dist = (receivePosition() - robot->pos).mag();
	float distAngle = Line(receivePosition(), ball().pos).distTo(robot->pos);

	return std::max(dist, distAngle);
}

bool Gameplay::DumbReceive::isAtReceivePosition()
{
	return receivePositionError() < *_maxExecutionError;
}

bool Gameplay::DumbReceive::run()
{
	using namespace Geometry2d;

	if(!robot || !robot->visible) {
		return false;
	}

	// State changing
	if(_state == Setup) {
		if(isAtReceivePosition()) {
			if(partner && partner->isActing()) {
				_state = Receive_PassKicking;
			} else if(partner && partner->isDone()) {
				_state = Receive_PassDone;
			}
			// else if(!partner) {
			// 	_state = Receive_PassDone;
			// }
		}
	} else if(_state == Receive_PassKicking) {
		if(partner && partner->isSettingUp()) {
			_state = Setup;
		} else if(partner && partner->isDone()) {
			_state = Receive_PassDone;
		} else if(!partner) {
			_state = Receive_PassDone;
		}
	} else if(_state == Receive_PassDone) {
		if(_kickDelta == 0) {
			_kickDelta = timestamp();
		}

		if(robot->hasBall()) {
			_state = Done;
		} else if(timestamp() - _kickDelta > *_kickTimeout * 1000000) {  // Micro-seconds
			_state = Done;
		}
	} else if(_state == Done) {
		// Do nothing
	}

	// Debug visualization
	if(*_debug) {
		if(_state == Setup) {
			robot->addText(QString("DR: Setup"), Qt::yellow, QString("DumbReceive"));
			robot->addText(QString("DR error: %1").arg(receivePositionError()), Qt::yellow, QString("DumbReceive"));
			// Draw circle at pass position
			state()->drawCircle(receivePosition(), Robot_Radius + 0.05, Qt::yellow, QString("DumbReceive"));
		} else if(_state == Receive_PassKicking) {
			robot->addText(QString("DR: Receive PassKicking"), Qt::yellow, QString("DumbReceive"));
			// Draw circle at pass position
			state()->drawCircle(receivePosition(), Robot_Radius + 0.05, Qt::yellow, QString("DumbReceive"));
			robot->addText(QString("DR error: %1").arg(receivePositionError()), Qt::yellow, QString("DumbReceive"));
			// Draw line to show where we think the ball is going to travel
			state()->drawLine(ball().pos, receivePosition(), Qt::yellow, QString("DumbReceive"));
		} else if(_state == Receive_PassDone) {
			robot->addText(QString("DR: Receive PassDone"), Qt::green, QString("DumbReceive"));
			robot->addText(QString("DR error: %1").arg(receivePositionError()), Qt::green, QString("DumbReceive"));
			// Draw circle at pass position
			state()->drawCircle(receivePosition(), Robot_Radius + 0.05, Qt::green, QString("DumbReceive"));
			// Draw line to show where we think the ball is going to travel
			state()->drawLine(ball().pos, receivePosition(), Qt::green, QString("DumbReceive"));
		} else if(_state == Done) {
			// Do nothing
			robot->addText(QString("DR: Done"), Qt::red, QString("DumbReceive"));
		}
	}

	// Commands
	if(_state == Setup) {
		robot->resetAvoidBall();
		robot->move(receivePosition(), false);
		robot->face(ball().pos);
	} else if(_state == Receive_PassKicking || _state == Receive_PassDone) {
		// FIXME: This has the side effect of always being on (even after play termination)
		// because it modifies an OurRobot internal variable.
		robot->disableAvoidBall();
		robot->move(receivePosition(), false);
		robot->face(ball().pos);
	}


	if ( _state != Receive_PassKicking ) {
		timeout.reset();
	} 
	// else if ( timeout.isTimedOut() ) {
	// 	_state = Receive_PassDone;
	// 	robot->addText(QString("DR: Timed out"), Qt::red, QString("DumbReceive"));
	// }

	return _state != Receive_PassDone;
}
