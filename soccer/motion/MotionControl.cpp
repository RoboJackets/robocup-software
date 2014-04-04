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

Point gearRatio;

ConfigDouble *MotionControl::_pid_pos_p;
ConfigDouble *MotionControl::_pid_pos_i;
ConfigDouble *MotionControl::_pid_pos_d;
ConfigDouble *MotionControl::_vel_mult;

ConfigDouble *MotionControl::_pid_angle_p;
ConfigDouble *MotionControl::_pid_angle_i;
ConfigDouble *MotionControl::_pid_angle_d;
ConfigDouble *MotionControl::_angle_vel_mult;
Point targetPos, targetVel;
ConfigDouble *MotionControl::_max_angle_w;

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
}


#pragma mark MotionControl

MotionControl::MotionControl(OurRobot *robot) : _angleController(0, 0, 0, 50) {
	_robot = robot;

	_robot->radioTx.set_robot_id(_robot->shell());
}



//	FIXME: we should use RobotDynamics instead
static const float Max_Linear_Speed = 0.008 * 511;
// static const float Max_Angular_Speed = 511 * 0.02 * M_PI;



// void limitAccel(float &value, float last, float limit)
// {
// //	float old = value;
// 	if (value > 0 && (value - last) > limit)
// 	{
// 		value = last + limit;
// 	} else if (value < 0 && (last - value) > limit)
// 	{
// 		value = last - limit;
// 	}
// }

// 	// Acceleration limit
// //	Point dv = bodyVel - _lastBodyVel;
// //	float dw = angularVel - _lastAngularVel;
	
// //	float av = *_robot->config->trapTrans.acceleration;
// //	float aw = *_robot->config->trapRot.acceleration;
	
// // 	limitAccel(bodyVel.x, _lastBodyVel.x, av);
// // 	limitAccel(bodyVel.y, _lastBodyVel.y, av);
// // 	limitAccel(angularVel, _lastAngularVel, aw);



void MotionControl::run() {
	if (!_robot) return;

	//	FIXME: acceleration limiting?

	//	FIXME: reset controller when we set a new path?

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

	//	if no target position is given, we don't have a path to follow
	if (!constraints.targetPos) {
		if (!constraints.targetWorldVel) {
			_robot->radioTx.set_body_x(0);
			_robot->radioTx.set_body_y(0);
		} else {
			Point velRotated = constraints.targetWorldVel->rotated(-_robot->angle);
			_robot->radioTx.set_body_x(velRotated.x);
			_robot->radioTx.set_body_y(velRotated.y);
		}
	} else {
		Point velRotated = _robot->vel.rotated(-_robot->angle);
		gearRatio = velRotated/targetVel;
		//
		//	Path following
		//

		//	convert from microseconds to seconds
		float timeIntoPath = (float)((timestamp() - _robot->pathStartTime()) / 1000000.0f);

		//	evaluate path - where should we be right now?
		
		bool pathValidNow = _robot->path().evaluate(timeIntoPath, targetPos, targetVel);
		_robot->addText(QString("targetVel %1 %2").arg(targetVel.x).arg(targetVel.y) );
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
		_robot->addText(QString("ratio %1 %2").arg(gearRatio.x).arg(gearRatio.y));
		//convert from world to body coordinates
		targetVel = targetVel.rotated(-_robot->angle);
		
		//	set radioTx values
		_robot->radioTx.set_body_x(targetVel.x);
		_robot->radioTx.set_body_y(targetVel.y);
	}
}
