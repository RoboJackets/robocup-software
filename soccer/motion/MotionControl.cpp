#include "MotionControl.hpp"
#include <framework/SystemState.hpp>
#include <framework/RobotConfig.hpp>
#include <Robot.hpp>
#include <Utils.hpp>

#include <cmath>
#include <stdio.h>
#include <algorithm>

using namespace std;
using namespace Utils;
using namespace Geometry2d;

MotionControl::MotionControl(OurRobot *robot)
{
	_robot = robot;

	_robot->radioTx.set_robot_id(_robot->shell());
	for (int i = 0; i < 4; ++i)
	{
		_robot->radioTx.add_motors(0);
	}
	
	_lastOutputSpeed = 0;
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
	_robot->addText(QString("curSpeed %1 %2 wanted %3").arg(curSpeed).arg(velocity.mag()).arg(_lastOutputSpeed));
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
	
	_lastOutputSpeed = newSpeed;
	_robot->cmd.worldVel = posErrorDir * newSpeed;
}

void MotionControl::positionPD()
{
	if (!_robot->cmd.target)
	{
		return;
	}
	
	static const float Latency = 0.176;
	
	float curSpeed = _robot->vel.mag();
	float maxSpeed = curSpeed + *_robot->config->trapTrans.acceleration * Latency;
	float cruise = *_robot->config->trapTrans.velocity;
	maxSpeed = min(maxSpeed, cruise);
	
	Point posError = _robot->cmd.target->pos - _robot->pos;
	float p = *_robot->config->translation.p;
	
	Point newVel = posError * p + (posError - _lastPosError) * *_robot->config->translation.d;
	float newSpeed = newVel.mag();
	if (newSpeed)
	{
		_robot->addText(QString().sprintf("Dist %f Speed %f %f", posError.mag(), curSpeed, newSpeed));
// 		_robot->addText(QString().sprintf("Range %f %f", minSpeed, maxSpeed));
		if (newSpeed > maxSpeed && newSpeed > 1.0)
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
	
	float error = Utils::fixAngleRadians(dir.angle() - _robot->angle * DegreesToRadians);
// 	float error = Utils::fixAngleRadians(M_PI * *_robot->config->rotation.i - _robot->angle * DegreesToRadians);
	
	_robot->cmd.angularVelocity = error * *_robot->config->rotation.p + (error - _lastAngleError) * *_robot->config->rotation.d;
	_lastAngleError = error;
}

void MotionControl::stopped()
{
}

void MotionControl::run()
{
	//FIXME - These are all 2011 numbers
	//FIXME set these through parameters/config
	
	// The largest wheel command that can be transmitted
	const int Max_Wheel_Command = 127;
	
	// How fast the robot speed control loop runs
	// (speeds are measured on the robot in encoder ticks/iteration)
	const int Robot_Control_Rate = 200;
	
	// Encoder ticks per revolution
	const int Ticks_Per_Rev = 1440;
	
	// Speed command scale for the radio protocol.
	// This reduces the maximum speed to a number that will fit in the forward packet.
	const int Protocol_Scale = 3;
	
	const double Gear_Ratio = 4.5;
	
	// The wheel angular velocity (rad/s) resulting from the largest wheel command
	const float Max_Wheel_Speed = 2 * M_PI * Max_Wheel_Command * Protocol_Scale * Robot_Control_Rate / (Ticks_Per_Rev * Gear_Ratio);
	
	// Effective wheel radius (m)
	const float Wheel_Radius = 0.026;
	
	// Maximum linear speed without rotation (m/s)
	const float Max_Linear_Speed = Max_Wheel_Speed * Wheel_Radius;
	
	// Radius of circle containing the roller contact points (m)
	const float Contact_Circle_Radius = 0.0812;
	
	// Maximum angular speed without translation
	const float Max_Angular_Speed = Max_Linear_Speed / Contact_Circle_Radius;
	
	// Axle direction, pointing out of the robot
	const Point axles[4] =
	{
		Point(-1,  1).normalized(),
		Point( 1,  1).normalized(),
		Point( 1, -1).normalized(),
		Point(-1, -1).normalized()
	};
	
// 	_velocity = _robot->pos - _lastPos;
	
// 	float dtime = (_robot->state()->timestamp - _lastFrameTime) / 1.0e6;
// 	_angularVelocity = fixAngleRadians((_robot->angle - _lastAngle) * DegreesToRadians) / dtime;
	
// 	positionTrapezoidal();
	positionPD();
	anglePD();
	
	// Scaling
	if (_robot->cmd.worldVel)
	{
		_robot->cmd.worldVel = *_robot->cmd.worldVel * _robot->cmd.vScale;
		if (!_robot->cmd.bodyVel)
		{
			_robot->cmd.bodyVel = _robot->cmd.worldVel->rotated(-_robot->angle);
		}
	}
	if (!_robot->cmd.angularVelocity)
	{
		_robot->cmd.angularVelocity = 0;
	}
	float scaledSpin = min(*_robot->cmd.angularVelocity * _robot->cmd.wScale, Max_Angular_Speed);
	
	if (!_robot->cmd.bodyVel)
	{
		_robot->cmd.bodyVel = Point();
	}
	
	Point &bodyVel = *_robot->cmd.bodyVel;
	
	// Sanity check
	if (!isfinite(scaledSpin) || !isfinite(bodyVel.x) || !isfinite(bodyVel.y))
	{
		_robot->addText(QString().sprintf("Non-normal motion results: rotate %f, translate %f, %f\n", scaledSpin, bodyVel.x, bodyVel.y));
		scaledSpin = 0;
		bodyVel = Point();
	}

	// position control through vision

	// Limit speed so we at least go in the right direction
	float s = bodyVel.mag();
	if (s > Max_Linear_Speed)
	{
		bodyVel = bodyVel / s * Max_Linear_Speed;
	}
	
	// Set motor commands for linear motion
	int maxPos = 0;
	int maxNeg = 0;
	int out[4];
	for (int i = 0; i < 4; ++i)
	{
		float w = axles[i].dot(bodyVel) / Wheel_Radius;
		int c = roundf(Max_Wheel_Command * w / Max_Wheel_Speed);
		
		// Keep track of the largest motor commands before clipping
		if (c > 0 && c > maxPos)
		{
			maxPos = c;
		} else if (c < 0 && c < maxNeg)
		{
			maxNeg = c;
		}
		
		// Clipping
		c = max(-Max_Wheel_Command, c);
		c = min(Max_Wheel_Command, c);
		
		out[i] = c;
	}
	
	// Get the largest motor command that was actually set
	maxPos = min(maxPos, Max_Wheel_Command);
	maxNeg = max(maxNeg, -Max_Wheel_Command);
	
	// Add as much rotation as we can without overflow
	int spinCommand = int(Max_Wheel_Command * scaledSpin / Max_Angular_Speed + 0.5);
	
	if (spinCommand > 0 && (maxPos + spinCommand) > Max_Wheel_Command)
	{
		spinCommand = Max_Wheel_Command - maxPos;
	} else if (spinCommand < 0 && (maxNeg + spinCommand) < -Max_Wheel_Command)
	{
		spinCommand = -Max_Wheel_Command - maxNeg;
	}
	
	for (int i = 0; i < 4; ++i)
	{
		out[i] += spinCommand;
	}

	// Store motor commands in the radio sub-packet
	for (int i = 0; i < 4; ++i)
	{
		_robot->radioTx.set_motors(i, out[i]);
	}
	
	_lastFrameTime = _robot->state()->timestamp;
	_lastPos = _robot->pos;
	_lastAngle = _robot->angle;
}
