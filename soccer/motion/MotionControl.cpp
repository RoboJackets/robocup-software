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

void MotionControl::createConfiguration(Configuration *cfg) {
	_pid_pos_p = new ConfigDouble(cfg, "MotionControl/pos/PID_p", 6.5);
	_pid_pos_i = new ConfigDouble(cfg, "MotionControl/pos/PID_i", 0.0001);
	_pid_pos_d = new ConfigDouble(cfg, "MotionControl/pos/PID_d", 2);
	_vel_mult = new ConfigDouble(cfg, "MotionControl/pos/Velocity Multiplier", 1);

	_pid_angle_p	= new ConfigDouble(cfg, "MotionControl/angle/PID_p");
	_pid_angle_i	= new ConfigDouble(cfg, "MotionControl/angle/PID_i");
	_pid_angle_d	= new ConfigDouble(cfg, "MotionControl/angle/PID_d");
	_angle_vel_mult	= new ConfigDouble(cfg, "MotionControl/angle/Velocity Multiplier");
}


#pragma mark MotionControl

MotionControl::MotionControl(OurRobot *robot) {
	_robot = robot;

	_robot->radioTx.set_robot_id(_robot->shell());
}



//	FIXME: we should use RobotDynamics instead
static const float Max_Linear_Speed = 0.008 * 511;
static const float Max_Angular_Speed = 511 * 0.02 * M_PI;



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

		//	use bang-bang to get from our current angle to the target angle

		float targetAngleFinal = (*constraints.faceTarget - _robot->pos).angle();
		float angleDiff = targetAngleFinal - _robot->angle;

		float unused, targetW;	//	FIXME: trapezoid
		TrapezoidalMotion(
			angleDiff,
			1,					//	FIXME: max angular speed
			2,					//	FIXME: set max angular acc
			.001,				//	time into trapezoid
			_robot->angleVel,	//	current angular vel
			0,					//	final angular speed of zero
			unused,				//	target pos out (unused)
			targetW);			//	according to trapezoid, the speed we should be going right now

		//	w multiplier
		targetW *= *_angle_vel_mult;

		//	PID on angle
		float angleError = 0;	//	TODO: actually take angle error into account
		targetW += _angleController.run(angleError);

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
			_robot->radioTx.set_body_x(constraints.targetWorldVel->x);
			_robot->radioTx.set_body_y(constraints.targetWorldVel->y);
		}
	} else {
		//
		//	Path following
		//

		//	FIXME: make sure there's a legit path

		//	convert from microseconds to seconds
		float timeIntoPath = (float)((timestamp() - _robot->pathStartTime()) / 1000000.0f);

		//	evaluate path - where should we be right now?
		Point targetPos, targetVel;
		bool pathValidNow = _robot->path().evaluate(timeIntoPath, targetPos, targetVel);

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
		_robot->state()->drawLine(targetPos, targetPos + targetVel
				, Qt::blue, "velocity");
		_robot->state()->drawText(QString("%1").arg(timeIntoPath), targetPos, Qt::black, "time");

		//convert from world to body coordinates
		targetVel = targetVel.rotated(-_robot->angle);
		//	set radioTx values
		_robot->radioTx.set_body_x(targetVel.x);
		_robot->radioTx.set_body_y(targetVel.y);

		
	}
}
