#include "MotionControl.hpp"
#include <framework/SystemState.hpp>
#include <framework/RobotConfig.hpp>
#include <Robot.hpp>
#include <Utils.hpp>

#include <cmath>
#include <stdio.h>
#include <algorithm>

using namespace std;
using namespace Geometry2d;

MotionControl::MotionControl(OurRobot *robot)
{
	_robot = robot;

	_robot->radioTx.set_robot_id(_robot->shell());
	
	_lastAngularVel = 0;
}

void MotionControl::positionTrapezoidal()
{
	if (!_robot->cmd.target)
	{
		return;
	}
	
	// The velocity controller assumes that the robot is capable of infinite acceleration,
	// and thus the speed command will be executed completely (+noise) by the next frame.
	
	const RobotConfig::Dynamics &config = _robot->config->trapTrans;
	float maxSpeed = *config.velocity;
	float acceleration = *config.acceleration;
// 	float deceleration = *config.deceleration;
	float predictTime = *config.predictTime;
	float responseTime = *config.responseTime;
	
	Point velocity = _robot->vel;
	Point predictPos = _robot->pos + velocity * predictTime;
	Point posError = _robot->cmd.target->pos - predictPos;
	Point posErrorDir = posError.normalized();
	
// 	float curSpeed = velocity.dot(posErrorDir);
	float curSpeed = _robot->vel.mag();
// 	float targetSpeed = 0;	//FIXME - If nonzero, more cases appear
	
	// Distance left to travel to reach goal
	float distanceLeft = posError.mag();
	
	// How far would we travel if we decelerated at the limit?
// 	float minStoppingDistance = (curSpeed * curSpeed - targetSpeed * targetSpeed) / (2 * deceleration);
	
	// The speed we need to stop at the goal with unlimited acceleration
	float stoppingSpeed = distanceLeft / responseTime;
	
	float newSpeed;
	
// 	_robot->addText(QString("minStoppingDistance %1 %2").arg(minStoppingDistance).arg(distanceLeft));
	_robot->addText(QString("curSpeed %1 %2").arg(curSpeed).arg(velocity.mag()));
	if (stoppingSpeed < curSpeed && stoppingSpeed < maxSpeed)
	{
		// Try to stop on the target regardless of deceleration
		newSpeed = stoppingSpeed;
		_robot->addText(QString("decel to %1").arg(newSpeed));
// 	} else if (distanceLeft <= minStoppingDistance * 1.5)
// 	{
// 		// Decelerate
// 		newSpeed = max(0.0f, curSpeed - deceleration * responseTime);
// 		_robot->addText(QString("decel to %1").arg(newSpeed));
	} else if (curSpeed < maxSpeed)
	{
		// Accelerate
		newSpeed = min(maxSpeed, curSpeed + acceleration * responseTime);
		newSpeed = min(newSpeed, stoppingSpeed);
		_robot->addText(QString("accel to %1").arg(newSpeed));
	} else {
		// Constant velocity
		newSpeed = maxSpeed;
		_robot->addText(QString("cruise at %1").arg(newSpeed));
	}
	
	_robot->cmd.worldVel = posErrorDir * newSpeed;
}

void MotionControl::positionPD()
{
	if (!_robot->cmd.target)
	{
		return;
	}
	
	float curSpeed = _robot->vel.mag();
	float maxSpeed = curSpeed + *_robot->config->trapTrans.acceleration;
	float cruise = *_robot->config->trapTrans.velocity;
	maxSpeed = min(maxSpeed, cruise);
	
	Point posError = _robot->cmd.target->pos - _robot->pos;
	float p = *_robot->config->translation.p;
	float d = *_robot->config->translation.d;
	
	Point differentialError = (posError - _lastPosError) / (_robot->state()->timestamp - _lastFrameTime);

	Point newVel =  ( posError * p ) + ( differentialError * d );

	Point deltaVel = (newVel - _robot->vel);
	float accel = deltaVel.mag();
	float maxAccel = *_robot->config->trapTrans.acceleration;
	if(accel > maxAccel)
	{
		deltaVel = deltaVel / accel * maxAccel;
	}
	newVel = _robot->vel + deltaVel;

	float newSpeed = newVel.mag();

	if (newSpeed)
	{
		_robot->addText(QString().sprintf("Dist %f Speed cur %f new %f max %f", posError.mag(), curSpeed, newSpeed, maxSpeed));
// 		_robot->addText(QString().sprintf("Range %f %f", minSpeed, maxSpeed));
		if (newSpeed > maxSpeed)
		{
			_robot->addText("Limited by accel/cruise");
			_robot->cmd.worldVel = newVel / newSpeed * maxSpeed;
		} else {
// 			_robot->addText("Unchanged");
			_robot->cmd.worldVel = newVel;
		}
		_robot->addText(QString().sprintf("Final vel %f %f", _robot->cmd.worldVel->x, _robot->cmd.worldVel->y));
	} else {
		// Not trying to move
		_robot->cmd.worldVel = Point();
	}
	
	_lastPosError = posError;
}

void MotionControl::anglePD()
{
	if (!_robot->cmd.face)
	{
		return;
	}
	
// 	_robot->state()->drawLine(_robot->pos, _robot->cmd.goalOrientation, Qt::black, "Motion");
	Point dir = (_robot->cmd.face->pos - _robot->pos).normalized();
	
	float error = fixAngleRadians(dir.angle() - _robot->angle * DegreesToRadians);
// 	float error = fixAngleRadians(M_PI * *_robot->config->rotation.i - _robot->angle * DegreesToRadians);
	
	_robot->cmd.angularVelocity = error * (float)*_robot->config->rotation.p + (error - _lastAngleError) * (float)*_robot->config->rotation.d;
	_lastAngleError = error;
}

void MotionControl::stopped()
{
}

void limitAccel(float &value, float last, float limit)
{
//	float old = value;
	if (value > 0 && (value - last) > limit)
	{
		value = last + limit;
	} else if (value < 0 && (last - value) > limit)
	{
		value = last - limit;
	}
// 	printf("limit %f %f %f -> %f\n", old, last, limit, value);
}

void MotionControl::run()
{
	static const float Max_Linear_Speed = 0.008 * 511;
	static const float Max_Angular_Speed = 511 * 0.02 * M_PI;
	
// 	positionTrapezoidal();
	positionPD();
	anglePD();
	
	// Scaling
	if (_robot->cmd.worldVel)
	{
// 		_robot->cmd.worldVel = *_robot->cmd.worldVel * _robot->cmd.vScale;
		if (!_robot->cmd.bodyVel)
		{
			_robot->cmd.bodyVel = _robot->cmd.worldVel->rotated(-_robot->angle);
		}
	}
	if (!_robot->cmd.angularVelocity)
	{
		_robot->cmd.angularVelocity = 0;
	}
	float angularVel = min(*_robot->cmd.angularVelocity * _robot->cmd.wScale, Max_Angular_Speed);
	
	if (!_robot->cmd.bodyVel)
	{
		_robot->cmd.bodyVel = Point();
	}
	
	Point &bodyVel = *_robot->cmd.bodyVel;
	
	// Sanity check
	if (!isfinite(angularVel) || !isfinite(bodyVel.x) || !isfinite(bodyVel.y))
	{
		_robot->addText(QString().sprintf("Non-normal motion results: rotate %f, translate %f, %f\n", angularVel, bodyVel.x, bodyVel.y));
		angularVel = 0;
		bodyVel = Point();
	}

	// position control through vision

	//FIXME - Find limits based on translational direction and speed

	// Limit speed so we at least go in the right direction
	float s = bodyVel.mag();
	if (s > Max_Linear_Speed)
	{
		bodyVel = bodyVel / s * Max_Linear_Speed;
	}
	
	// Acceleration limit
//	Point dv = bodyVel - _lastBodyVel;
//	float dw = angularVel - _lastAngularVel;
	
//	float av = *_robot->config->trapTrans.acceleration;
//	float aw = *_robot->config->trapRot.acceleration;
	
// 	limitAccel(bodyVel.x, _lastBodyVel.x, av);
// 	limitAccel(bodyVel.y, _lastBodyVel.y, av);
// 	limitAccel(angularVel, _lastAngularVel, aw);
	
	_lastBodyVel = bodyVel;
	_lastAngularVel = angularVel;
	
	_robot->radioTx.set_body_x(bodyVel.x);
	_robot->radioTx.set_body_y(bodyVel.y);
	_robot->radioTx.set_body_w(angularVel);
	
	_lastFrameTime = _robot->state()->timestamp;
	_lastPos = _robot->pos;
	_lastAngle = _robot->angle;
}
