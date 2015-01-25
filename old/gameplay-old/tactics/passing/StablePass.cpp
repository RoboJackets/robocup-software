#include "StablePass.hpp"
#include <Utils.hpp>
#include <math.h>

namespace Gameplay { REGISTER_CONFIGURABLE(StablePass) }

ConfigDouble *Gameplay::StablePass::_ERROR_PASSPOSITION;
ConfigDouble *Gameplay::StablePass::_ERROR_PASSDIRECTION;
ConfigDouble *Gameplay::StablePass::_ERROR_PASSPATH;
ConfigInt *Gameplay::StablePass::_KICKPOWER;
ConfigDouble *Gameplay::StablePass::_BACKOFF_DISTANCE;
ConfigDouble *Gameplay::StablePass::_APPROACH_VELOCITY;
ConfigBool *Gameplay::StablePass::_DEBUG;

using namespace Geometry2d;

void Gameplay::StablePass::createConfiguration(Configuration *cfg)
{
	_ERROR_PASSPOSITION = new ConfigDouble(cfg, "StablePass/Pass Position Error", 0.015);
	_ERROR_PASSDIRECTION = new ConfigDouble(cfg, "StablePass/Pass Direction Error", 0.015);
	_ERROR_PASSPATH = new ConfigDouble(cfg, "StablePass/Pass Path Error", 0.25);

	_KICKPOWER = new ConfigInt(cfg, "StablePass/Kick Power", 255);
	_BACKOFF_DISTANCE = new ConfigDouble(cfg, "StablePass/Backoff Distance", 0.2);
	_APPROACH_VELOCITY = new ConfigDouble(cfg, "StablePass/Approach Velocity", 1.0);

	_DEBUG = new ConfigBool(cfg, "StablePivotKick/Debug", true);
}

Gameplay::StablePass::StablePass(GameplayModule *gameplay)
	: ActionBehavior(gameplay),
	  _state(Setup)
{
}

bool Gameplay::StablePass::isSettingUp()
{
	return _state == Setup;
}

bool Gameplay::StablePass::isSetup()
{
	return ok_Pass() && _state == Setup;
}

bool Gameplay::StablePass::isActing()
{
	return _state == Kick;
}

bool Gameplay::StablePass::isDone()
{
	return _state == Done;
}

void Gameplay::StablePass::restart()
{
	_state = Setup;
}

Geometry2d::Point Gameplay::StablePass::compute_PassTarget()
{
	return actionTarget;
}

Geometry2d::Point Gameplay::StablePass::compute_PassDirection()
{
	return (compute_PassTarget() - ball().pos).normalized();
}

Geometry2d::Point Gameplay::StablePass::compute_PassPosition()
{
	Point ball_pos = ball().pos;
	Point pass_dir = compute_PassDirection();
	return ball_pos - pass_dir*(*_BACKOFF_DISTANCE);
}

double Gameplay::StablePass::error_PassPosition()
{
	return compute_PassPosition().distTo(robot->pos);
}

double Gameplay::StablePass::error_PassDirection()
{
	Point pass_dir = compute_PassDirection();
	Point target = compute_PassTarget();

	Line pass_line_perp(target, target + pass_dir.perpCW());

    state()->drawLine(pass_line_perp, Qt::red, QString("asldkjf"));

    Point kick_dir = (robot->kickerBar().center() - robot->pos).normalized();
	Line kick_line(ball().pos, ball().pos + kick_dir);

    state()->drawLine(kick_line, Qt::red, QString("StablePass"));

	Point kick_pt;
    pass_line_perp.intersects(kick_line, &kick_pt);

	state()->drawCircle(kick_pt, 0.05, Qt::red, QString("Error: Pass Direction"));
    robot->addText(QString("kick pt = %1, %2").arg(kick_pt.x).arg(kick_pt.y));

	return (target - kick_pt).mag();
}

double Gameplay::StablePass::error_PassPath()
{
	// The intuition here is to stay on the path and stay near the ball
	double error_path = Line(compute_PassPosition(), compute_PassTarget()).distTo(robot->pos);
	double error_ball = (ball().pos - robot->pos).mag();

	return std::max(error_path, error_ball);
}

bool Gameplay::StablePass::ok_PassPosition()
{
	return error_PassPosition() < *_ERROR_PASSPOSITION;
}

bool Gameplay::StablePass::ok_PassDirection()
{
	return error_PassDirection() < *_ERROR_PASSDIRECTION;
}

bool Gameplay::StablePass::ok_Pass()
{
	return ok_PassPosition() && ok_PassDirection();
}

bool Gameplay::StablePass::ok_PassPath()
{
	return error_PassPath() < *_ERROR_PASSPATH;
}

bool Gameplay::StablePass::ok_PassApproach()
{
	return ok_PassPath() && ok_PassDirection();
}

bool Gameplay::StablePass::run()
{
	using namespace Geometry2d;

	if(!robot || !robot->visible) {
		return false;
	}

	switch (_state) {
	case Setup:
		if(ok_Pass()) {
			if(!partner || partner->isSetup()) {
				_state = Kick;
			}
		}
		break;
	case Kick:
		if(robot->hasBall()) {
			_state = Done;
		} else if(!ok_PassPath()) {
			_state = Setup;
		}
		break;
	case Done:
		// Do Nothing
		break;
	default: break;
	}

	// Debug visualization
	if(*_DEBUG) {
		if(_state == Setup) {

			robot->addText(QString("SP: Setup"), Qt::yellow, QString("StablePass"));
			robot->addText(QString("SP: pos err= %1").arg(error_PassPosition()), Qt::yellow, QString("StablePass"));
			robot->addText(QString("SP: dir err= %1").arg(error_PassDirection()), Qt::yellow, QString("StablePass"));
			// Draw circle at pass position
			state()->drawCircle(compute_PassPosition(), Robot_Radius + 0.05, Qt::yellow, QString("StablePass"));

			// Draw line to show where robot is going to kick
			float targetDist = (compute_PassTarget() - ball().pos).mag() + Robot_Radius;
			Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
			state()->drawLine(robot->pos, robot->pos + faceVector*targetDist, Qt::yellow, QString("StablePass"));

            state()->drawCircle(compute_PassTarget(), 0.01, Qt::black, QString("StablePass"));

		} else if(_state == Kick) {

			robot->addText(QString("SP: Kick"), Qt::green, QString("StablePass"));
			robot->addText(QString("SP: path err= %1").arg(error_PassPath()), Qt::green, QString("StablePass"));
			robot->addText(QString("SP: dir err= %1").arg(error_PassDirection()), Qt::green, QString("StablePass"));

			// Draw line to show where robot is going to kick
			float targetDist = (actionTarget - ball().pos).mag() + Robot_Radius;
			Point faceVector = (robot->kickerBar().center() - robot->pos).normalized();
			state()->drawLine(robot->pos, robot->pos + faceVector*targetDist, Qt::green, QString("StablePass"));

		} else if(_state == Done) {

			// Do nothing
			robot->addText(QString("SP: Done"), Qt::red, QString("StablePass"));

		}
	}

	// Commands
	if(_state == Setup) {

		robot->resetAvoidBall();
		robot->move(compute_PassPosition(), false);
		robot->face(robot->pos + compute_PassDirection());

	} else if(_state == Kick) {

		robot->kick(*_KICKPOWER);

		// FIXME: This has the side effect of always being on, even after play termination
		// because it modifies a OurRobot internal variable.
		robot->disableAvoidBall();

		// Move towards receive target to kick the ball on that path.
		robot->move(ball().pos, false);
		// Point drive_dir = compute_PassDirection();

        // // Correct for errors when driving forward
        // Line drive_line(ball().pos, ball().pos + drive_dir);

		// robot->worldVelocity(drive_dir**_APPROACH_VELOCITY);
		robot->face(ball().pos);

	} else if(_state == Done) {

		robot->avoidBallRadius(2.0 * Ball_Radius);

	}


	return true;
}
