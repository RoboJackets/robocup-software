#include "StableLineKick.hpp"

namespace Gameplay { REGISTER_CONFIGURABLE(StableLineKick) }

ConfigInt *Gameplay::StableLineKick::_kickPower;
ConfigDouble *Gameplay::StableLineKick::_backoffDistance;
ConfigDouble *Gameplay::StableLineKick::_maxExecutionError;
ConfigDouble *Gameplay::StableLineKick::_approachVelocity;

ConfigBool *Gameplay::StableLineKick::_debug;

void Gameplay::StableLineKick::createConfiguration(Configuration *cfg)
{
	_kickPower = new ConfigInt(cfg, "StableLineKick/Kick Power", 100);
	_backoffDistance = new ConfigDouble(cfg, "StableLineKick/Backoff Distance", 0.2);
	_maxExecutionError = new ConfigDouble(cfg, "StableLineKick/Max Execution Error", 0.01);
	_approachVelocity = new ConfigDouble(cfg, "StableLineKick/Approach Velocity", 1.0);

	_debug = new ConfigBool(cfg, "StablePivotKick/Debug", true);
}

Gameplay::StableLineKick::StableLineKick(GameplayModule *gameplay)
	: PassingBehavior(gameplay),
	  _state(Setup)
{
}

bool Gameplay::StableLineKick::done()
{
	return _state == Done;
}

bool Gameplay::StableLineKick::kicking()
{
	return _state == Kick;
}

bool Gameplay::StableLineKick::setup()
{
	return _state == Setup;
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
	Point passVector = (receiveTarget - ballPos).normalized();
	Point passPos = ballPos - passVector**_backoffDistance;

	return passPos;
}

bool Gameplay::StableLineKick::isAtPassPosition()
{
	using namespace Geometry2d;
	float dist = passPosition().distTo(robot->pos);
	Line passLine(robot->pos, robot->kickerBar().center());
	float distAngle = passLine.distTo(receiveTarget);
	return dist < *_maxExecutionError && distAngle < *_maxExecutionError;
}

bool Gameplay::StableLineKick::isOnPassPath()
{
	using namespace Geometry2d;

	float distError = Line(passPosition(), receiveTarget).distTo(robot->pos);
	float distBall = (ball().pos - robot->pos).mag();
	return distError < *_maxExecutionError && distBall < *_backoffDistance;
}

bool Gameplay::StableLineKick::run()
{
	using namespace Geometry2d;

	if(!robot || !robot->visible) {
		return false;
	}

	// State changing
	if(_state == Setup) {
		if(isAtPassPosition()) {
			_state = Kick;
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
			// Draw circle at pass position
			state()->drawCircle(passPosition(), Robot_Radius + 0.05, Qt::yellow, QString("StableLineKick"));
			// Draw line to show where robot is going to kick
			float targetDist = (receiveTarget - ball().pos).mag() + Robot_Radius;
			Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
			state()->drawLine(robot->pos, robot->pos + faceVector*targetDist, Qt::yellow, QString("StableLineKick"));
		} else if(_state == Kick) {
			robot->addText(QString("SLK: Kick"), Qt::green, QString("StableLineKick"));
			// Draw line to show where robot is going to kick
			float targetDist = (receiveTarget - ball().pos).mag() + Robot_Radius;
			Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
			state()->drawLine(robot->pos, robot->pos + faceVector*targetDist, Qt::green, QString("StableLineKick"));
		} else if(_state == Done) {
			// Do nothing
			robot->addText(QString("SLK: Done"), Qt::red, QString("StableLineKick"));
		}
	}

	// Commands
	if(_state == Setup) {
		robot->avoidBall();
		robot->move(passPosition(), false);
		robot->face(ball().pos);
	} else if(_state == Kick) {
		robot->kick(*_kickPower);
		// FIXME: This has the side effect of always being on, even after play termination
		// because it modifies a OurRobot internal variable.
		robot->disableAvoidBall();
		// Move towards receive target to kick the ball on that path.
		//robot->move(receiveTarget, false);
		Point driveVector = (ball().pos - robot->pos).normalized();
		robot->worldVelocity(driveVector**_approachVelocity);
		robot->face(ball().pos);
	}


	return true;
}
