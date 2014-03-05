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

// void MotionControl::positionTrapezoidal()
// {
	
// 	// if (!_robot->cmd.target)
// 	// {
// 	// 	return;
// 	// }
	
// 	// The velocity controller assumes that the robot is capable of infinite acceleration,
// 	// and thus the speed command will be executed completely (+noise) by the next frame.
	
// 	const RobotConfig::Dynamics &config = _robot->config->trapTrans;
// 	float maxSpeed = *config.velocity;
// 	float acceleration = *config.acceleration;
// // 	float deceleration = *config.deceleration;
// 	float predictTime = *config.predictTime;
// 	float responseTime = *config.responseTime;
	
// 	Point velocity = _robot->vel;
// 	Point predictPos = _robot->pos + velocity * predictTime;
// 	Point posError(1,0);// = _robot->cmd.target->pos - predictPos;
// 	Point posErrorDir = posError.normalized();
	
// // 	float curSpeed = velocity.dot(posErrorDir);
// 	float curSpeed = _robot->vel.mag();
// // 	float targetSpeed = 0;	//FIXME - If nonzero, more cases appear
	
// 	// Distance left to travel to reach goal
// 	float distanceLeft = posError.mag();
	
// 	// How far would we travel if we decelerated at the limit?
// // 	float minStoppingDistance = (curSpeed * curSpeed - targetSpeed * targetSpeed) / (2 * deceleration);
	
// 	// The speed we need to stop at the goal with unlimited acceleration
// 	float stoppingSpeed = distanceLeft / responseTime;
	
// 	float newSpeed;
	
// // 	_robot->addText(QString("minStoppingDistance %1 %2").arg(minStoppingDistance).arg(distanceLeft));
// 	_robot->addText(QString("curSpeed %1 %2").arg(curSpeed).arg(velocity.mag()));
// 	if (stoppingSpeed < curSpeed && stoppingSpeed < maxSpeed)
// 	{
// 		// Try to stop on the target regardless of deceleration
// 		newSpeed = stoppingSpeed;
// 		_robot->addText(QString("decel to %1").arg(newSpeed));
// // 	} else if (distanceLeft <= minStoppingDistance * 1.5)
// // 	{
// // 		// Decelerate
// // 		newSpeed = max(0.0f, curSpeed - deceleration * responseTime);
// // 		_robot->addText(QString("decel to %1").arg(newSpeed));
// 	} else if (curSpeed < maxSpeed)
// 	{
// 		// Accelerate
// 		newSpeed = min(maxSpeed, curSpeed + acceleration * responseTime);
// 		newSpeed = min(newSpeed, stoppingSpeed);
// 		_robot->addText(QString("accel to %1").arg(newSpeed));
// 	} else {
// 		// Constant velocity
// 		newSpeed = maxSpeed;
// 		_robot->addText(QString("cruise at %1").arg(newSpeed));
// 	}
	
// 	// _robot->cmd.worldVel = posErrorDir * newSpeed;
	
// }

// void MotionControl::positionPD()
// {
	
// 	// if (!_robot->cmd.target)
// 	// {
// 	// 	return;
// 	// }
	
// 	float curSpeed = _robot->vel.mag();
// 	float maxSpeed = curSpeed + *_robot->config->trapTrans.acceleration;
// 	float cruise = *_robot->config->trapTrans.velocity;
// 	maxSpeed = min(maxSpeed, cruise);
	
// 	Point posError = Point(1, 0);//_robot->cmd.target->pos - _robot->pos;
// 	float p = *_robot->config->translation.p;
// 	float d = *_robot->config->translation.d;
	
// 	Point differentialError = (posError - _lastPosError) / (_robot->state()->timestamp - _lastFrameTime);

// 	Point newVel =  ( posError * p ) + ( differentialError * d );

// 	Point deltaVel = (newVel - _robot->vel);
// 	float accel = deltaVel.mag();
// 	float maxAccel = *_robot->config->trapTrans.acceleration;
// 	if(accel > maxAccel)
// 	{
// 		deltaVel = deltaVel / accel * maxAccel;
// 	}
// 	newVel = _robot->vel + deltaVel;

// 	float newSpeed = newVel.mag();

// 	if (newSpeed)
// 	{
// 		_robot->addText(QString().sprintf("Dist %f Speed cur %f new %f max %f", posError.mag(), curSpeed, newSpeed, maxSpeed));
// // 		_robot->addText(QString().sprintf("Range %f %f", minSpeed, maxSpeed));
// 		if (newSpeed > maxSpeed)
// 		{
// 			_robot->addText("Limited by accel/cruise");
// 			// _robot->cmd.worldVel = newVel / newSpeed * maxSpeed;
// 		} else {
// // 			_robot->addText("Unchanged");
// 			// _robot->cmd.worldVel = newVel;
// 		}
// 		// _robot->addText(QString().sprintf("Final vel %f %f", _robot->cmd.worldVel->x, _robot->cmd.worldVel->y));
// 	} else {
// 		// Not trying to move
// 		// _robot->cmd.worldVel = Point();
// 	}
	
// 	_lastPosError = posError;
	
// }

// void MotionControl::anglePD()
// {
	
// // 	if (!_robot->cmd.face)
// // 	{
// // 		return;
// // 	}
	
// // // 	_robot->state()->drawLine(_robot->pos, _robot->cmd.goalOrientation, Qt::black, "Motion");
// // 	Point dir = (_robot->cmd.face->pos - _robot->pos).normalized();
	
// // 	float error = fixAngleRadians(dir.angle() - _robot->angle * DegreesToRadians);
// // // 	float error = fixAngleRadians(M_PI * *_robot->config->rotation.i - _robot->angle * DegreesToRadians);
	
// // 	// _robot->cmd.angularVelocity = error * (float)*_robot->config->rotation.p + (error - _lastAngleError) * (float)*_robot->config->rotation.d;
// // 	_lastAngleError = error;
	
// }

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
// // 	printf("limit %f %f %f -> %f\n", old, last, limit, value);
// }

// void MotionControl::run()
// {
// 	static const float Max_Linear_Speed = 0.008 * 511;
// 	static const float Max_Angular_Speed = 511 * 0.02 * M_PI;
	
// // 	positionTrapezoidal();
// 	positionPD();
// 	anglePD();
	
// 	// Scaling
// // 	if (_robot->cmd.worldVel)
// // 	{
// // // 		_robot->cmd.worldVel = *_robot->cmd.worldVel * _robot->cmd.vScale;
// // 		if (!_robot->cmd.bodyVel)
// // 		{
// // 			_robot->cmd.bodyVel = _robot->cmd.worldVel->rotated(-_robot->angle);
// // 		}
// // 	}
// // 	if (!_robot->cmd.angularVelocity)
// // 	{
// // 		_robot->cmd.angularVelocity = 0;
// // 	}
// 	float angularVel = 1;//min(*_robot->cmd.angularVelocity * _robot->cmd.wScale, Max_Angular_Speed);
	
// 	// if (!_robot->cmd.bodyVel)
// 	// {
// 	// 	_robot->cmd.bodyVel = Point();
// 	// }
	
// 	Point bv(1,0);
// 	Point &bodyVel = bv;
	
// 	// Sanity check
// 	if (!isfinite(angularVel) || !isfinite(bodyVel.x) || !isfinite(bodyVel.y))
// 	{
// 		_robot->addText(QString().sprintf("Non-normal motion results: rotate %f, translate %f, %f\n", angularVel, bodyVel.x, bodyVel.y));
// 		angularVel = 0;
// 		bodyVel = Point();
// 	}

// 	// position control through vision

// 	//FIXME - Find limits based on translational direction and speed

// 	// Limit speed so we at least go in the right direction
// 	float s = bodyVel.mag();
// 	if (s > Max_Linear_Speed)
// 	{
// 		bodyVel = bodyVel / s * Max_Linear_Speed;
// 	}
	
// 	// Acceleration limit
// //	Point dv = bodyVel - _lastBodyVel;
// //	float dw = angularVel - _lastAngularVel;
	
// //	float av = *_robot->config->trapTrans.acceleration;
// //	float aw = *_robot->config->trapRot.acceleration;
	
// // 	limitAccel(bodyVel.x, _lastBodyVel.x, av);
// // 	limitAccel(bodyVel.y, _lastBodyVel.y, av);
// // 	limitAccel(angularVel, _lastAngularVel, aw);
	
// 	_lastBodyVel = bodyVel;
// 	_lastAngularVel = angularVel;
	
// 	_robot->radioTx.set_body_x(bodyVel.x);
// 	_robot->radioTx.set_body_y(bodyVel.y);
// 	_robot->radioTx.set_body_w(angularVel);
	
// 	_lastFrameTime = _robot->state()->timestamp;
// 	_lastPos = _robot->pos;
// 	_lastAngle = _robot->angle;
// }


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
			//	FIXME: what do if path invalid? stop?
		}

		//	tracking error
		Point posError = targetPos - _robot->pos;

		//	velocity multiplier
		targetVel *= *_vel_mult;

		//	PID on position
		targetVel.x += _positionXController.run(posError.x);
		targetVel.y += _positionYController.run(posError.y);

		//	draw target pt
		_robot->state()->drawCircle(targetPos, .04, Qt::blue, "MotionControl");

		//	set radioTx values
		_robot->radioTx.set_body_x(targetVel.x);
		_robot->radioTx.set_body_y(targetVel.y);
	}
}
