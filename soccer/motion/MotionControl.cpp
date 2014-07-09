#include "MotionControl.hpp"
#include <SystemState.hpp>
#include <RobotConfig.hpp>
#include <Robot.hpp>
#include <Utils.hpp>
#include "TrapezoidalMotion.hpp"

#include <cmath>
#include <stdio.h>
#include <algorithm>

using namespace std;
using namespace Geometry2d;


#pragma mark Config Variables

REGISTER_CONFIGURABLE(MotionControl);

ConfigDouble *MotionControl::_pid_pos_p;
ConfigDouble *MotionControl::_pid_pos_i;
ConfigDouble *MotionControl::_pid_pos_d;
ConfigDouble *MotionControl::_vel_mult;

ConfigDouble *MotionControl::_pid_angle_p;
ConfigDouble *MotionControl::_pid_angle_i;
ConfigDouble *MotionControl::_pid_angle_d;
ConfigDouble *MotionControl::_angle_vel_mult;
ConfigDouble *MotionControl::_max_angle_w;

ConfigDouble *MotionControl::_max_acceleration;
ConfigDouble *MotionControl::_max_velocity;

void MotionControl::createConfiguration(Configuration *cfg) {
	_pid_pos_p = new ConfigDouble(cfg, "MotionControl/pos/PID_p", 6.5);
	_pid_pos_i = new ConfigDouble(cfg, "MotionControl/pos/PID_i", 0.0001);
	_pid_pos_d = new ConfigDouble(cfg, "MotionControl/pos/PID_d", 2);
	_vel_mult = new ConfigDouble(cfg, "MotionControl/pos/Velocity Multiplier", 1);

	_pid_angle_p	= new ConfigDouble(cfg, "MotionControl/angle/PID_p", 1);
	_pid_angle_i	= new ConfigDouble(cfg, "MotionControl/angle/PID_i", 0.00001);
	_pid_angle_d	= new ConfigDouble(cfg, "MotionControl/angle/PID_d", 0.001);
	_angle_vel_mult	= new ConfigDouble(cfg, "MotionControl/angle/Velocity Multiplier");
	_max_angle_w	= new ConfigDouble(cfg, "MotionControl/angle/Max w", 10);

	_max_acceleration	= new ConfigDouble(cfg, "MotionControl/Max Acceleration", 1.5);
	_max_velocity		= new ConfigDouble(cfg, "MotionControl/Max Velocity", 2.0);
}


#pragma mark MotionControl

MotionControl::MotionControl(OurRobot *robot) : _angleController(0, 0, 0, 50) {
	_robot = robot;

	_robot->radioTx.set_robot_id(_robot->shell());
	_lastCmdTime = -1;
}


//	FIXME: we should use RobotDynamics instead
// static const float Max_Angular_Speed = 511 * 0.02 * M_PI;



void MotionControl::run() {
	if (!_robot) return;

	const MotionConstraints &constraints = _robot->motionConstraints();

	//	update PID parameters
	_positionXController.kp = *_pid_pos_p;
	_positionXController.ki = *_pid_pos_i;
	_positionXController.kd = *_pid_pos_d;
	_positionYController.kp = *_pid_pos_p;
	_positionYController.ki = *_pid_pos_i;
	_positionYController.kd = *_pid_pos_d;
	_angleController.kp = *_pid_angle_p;
	_angleController.ki = *_pid_angle_i;
	_angleController.kd = *_pid_angle_d;


	//	Angle control //////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////

	if (constraints.targetAngleVel) {
		float wMult = *_angle_vel_mult;
		_robot->radioTx.set_body_w((*constraints.targetAngleVel) * wMult);
	} else if (constraints.faceTarget) {

		//	FIXME: explain units: radians vs degrees

		//	TODO: use bang-bang to get from our current angle to the target angle?

		float targetAngleFinal = (*constraints.faceTarget - _robot->pos).angle() * 180.0 / M_PI;
		float angleError = targetAngleFinal - _robot->angle;

		while (angleError > 180) {
			angleError -= 360;
		} 
		while (angleError < -180) {
			angleError += 360;
		}


		float targetW;
		float targetAngle;
		TrapezoidalMotion(
			abs(angleError),	//	dist
			90,	//	max deg/sec
			30,	//	max deg/sec^2
			0.1 ,	//	time into path
			_robot->angleVel,
			0,		//	final speed
			targetAngle,
			targetW
			);


		targetW *= *_angle_vel_mult;

		//	PID on angle
		if(angleError<0) {
			targetW = - targetW;
		}
		targetW = _angleController.run(targetAngle);


		targetW = _angleController.run(angleError);
		if(abs(targetW) > (*_max_angle_w)) {
			if(targetW>0) {
				targetW = (*_max_angle_w);
			} else {
				targetW = -(*_max_angle_w);
			}
		}
	/*	_robot->addText(QString("targetW: %1").arg(targetW));
		_robot->addText(QString("angleError: %1").arg(angleError));
		_robot->addText(QString("targetGlobalAngle: %1").arg(targetAngleFinal));
		_robot->addText(QString("angle: %1").arg(_robot->angle));*/

		//	radio cmd
		_robot->radioTx.set_body_w(targetW);
	} else {
		_robot->radioTx.set_body_w(0);
	}


	//	Position control ///////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////

	Point targetVel;

	//	if no target position is given, we don't have a path to follow
	if (!_robot->path()) {
		if (!constraints.targetWorldVel) {
			targetVel = Point(0, 0);
		} else {
			targetVel = constraints.targetWorldVel->rotated(-_robot->angle);
		}
	} else {
		//
		//	Path following
		//
		
		
		//	convert from microseconds to seconds
		float timeIntoPath = ((float)(timestamp() - _robot->pathStartTime())) * TimestampToSecs;

		//	if the path is getting rapidly changed, we cheat so that the robot actually moves
		//	see OurRobot._recentPathChangeTimes for more info
		if (_robot->isRepeatedlyChangingPaths()) {
			timeIntoPath = max<float>(timeIntoPath, OurRobot::PathChangeHistoryBufferSize * 1.0f/60.0f * 0.8);
		}

		//	evaluate path - where should we be right now?
		Point targetPos;
		bool pathValidNow = _robot->path()->evaluate(
			timeIntoPath,
			targetPos,
			targetVel);
		if (!pathValidNow) {
			targetVel.x = 0;
			targetVel.y = 0;
		} 
		//	tracking error
		Point posError = targetPos - _robot->pos;

		//	velocity multiplier
		targetVel *= *_vel_mult;

		//	PID on position
		targetVel.x += _positionXController.run(posError.x);
		targetVel.y += _positionYController.run(posError.y);

		//	draw target pt
		_robot->state()->drawCircle(targetPos, .04, Qt::red, "MotionControl");
		_robot->state()->drawLine(targetPos, targetPos + targetVel, Qt::blue, "velocity");
		_robot->state()->drawText(QString("%1").arg(timeIntoPath), targetPos, Qt::black, "time");

		//	convert from world to body coordinates
		targetVel = targetVel.rotated(-_robot->angle);
	}

	this->_targetVel(targetVel);
}

void MotionControl::stopped() {
	_targetVel(Point(0, 0));
}

void MotionControl::_targetVel(Point targetVel) {
	// Limit Velocity
	targetVel.clamp(*_max_velocity);

	// Limit Acceleration
	if (_lastCmdTime == -1) {
		targetVel.clamp(*_max_acceleration);
	} else {
		float dt = (float)((timestamp() - _lastCmdTime) / 1000000.0f);
		Point targetAccel = (targetVel - _lastVelCmd) / dt ;
		targetAccel.clamp(*_max_acceleration);

		targetVel = _lastVelCmd + targetAccel * dt;
	}

	//	make sure we don't send any bad values
	if (isnan(targetVel.x) || isnan(targetVel.y)) {
		targetVel = Point(0,0);
	}

	//	set radioTx values
	_robot->radioTx.set_body_x(targetVel.x);
	_robot->radioTx.set_body_y(targetVel.y);

	//	track these values so we can limit acceleration
	_lastVelCmd = targetVel;
	_lastCmdTime = timestamp();
}
