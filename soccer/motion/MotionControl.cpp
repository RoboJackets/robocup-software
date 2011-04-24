#include "MotionControl.hpp"
#include <framework/RobotConfig.hpp>
#include <Robot.hpp>
#include <Utils.hpp>

#include <stdio.h>
#include <algorithm>

using namespace std;
using namespace Geometry2d;

MotionControl::MotionControl(OurRobot *robot)
{
	_robot = robot;

	_robot->radioTx.set_board_id(_robot->shell());
	for (int m = 0; m < 4; ++m)
	{
		_robot->radioTx.add_motors(0);
	}
}

void MotionControl::positionTrapezoidal()
{
	Point posError = _robot->cmd.goalPosition - _robot->pos;

	Point posErrorDir = posError.normalized();
	
	//FIXME - The velocity controller assumes that the robot is capable of infinite acceleration,
	// and thus the speed command will be executed completely (+noise) by the next frame.
	
	// Trapezoidal control
	const RobotConfig::Dynamics &config = _robot->config->trapTrans;
	float curSpeed = _robot->vel.mag();
	
	// Distance left to travel to reach goal
	float distanceLeft = posError.mag();
	
	// How far would we travel if we decelerated at the limit?
	float minStoppingTime = curSpeed / config.deceleration;
	float minStoppingDistance = curSpeed * minStoppingTime + 0.5 * config.deceleration * minStoppingTime * minStoppingTime;
	
	//FIXME - Frame time from Processor
	const float FrameTime = 1.0 / 60.0;
	
	float targetSpeed;
	
	_robot->addText(QString("minStoppingDistance %1 %2").arg(minStoppingDistance).arg(curSpeed));
	//FIXME - Predict speed changes over time
	if (distanceLeft <= minStoppingDistance)
	{
		// Decelerate
		//FIXME - Calculate the value necessary to reach the goal, regardless of limit
		float decel = config.deceleration;
		targetSpeed = max(0.0f, curSpeed - decel * FrameTime);
		_robot->addText(QString("decel to %1").arg(targetSpeed));
	} else if (curSpeed < config.velocity)
	{
		// Accelerate
		if (curSpeed <= 0.05)
		{
			targetSpeed = config.velocity;
		} else {
			targetSpeed = min((double)config.velocity, curSpeed + config.acceleration * FrameTime * 10);
		}
		_robot->addText(QString("accel to %1").arg(targetSpeed));
	} else {
		// Constant velocity
		targetSpeed = config.velocity;
		_robot->addText(QString("cruise at %1").arg(targetSpeed));
	}
	
	_worldVel = posErrorDir * targetSpeed;
}

void MotionControl::positionPD()
{
	float curSpeed = _robot->vel.mag();
	const float FrameTime = 1.0 / 60.0;
	float maxSpeed = curSpeed + _robot->config->trapTrans.acceleration;
	float cruise = _robot->config->trapTrans.velocity;
	maxSpeed = min(maxSpeed, cruise);
	float minSpeed = curSpeed - _robot->config->trapTrans.deceleration;
	
	float deadzone = 0.4f;
	if (_robot->hasBall)
	{
		deadzone = 0.7f;
	}
	
	minSpeed = max(deadzone, minSpeed);
	
	Point posError = _robot->cmd.goalPosition - _robot->pos;
	float p = _robot->config->translation.p;
	
	Point newVel = posError * p + (posError - _lastPosError) * _robot->config->translation.d;
	float newSpeed = newVel.mag();
	_robot->addText(QString().sprintf("Speed %f %f", curSpeed, newSpeed));
	_robot->addText(QString().sprintf("Range %f %f", minSpeed, maxSpeed));
	if (newSpeed > maxSpeed && newSpeed > 1.0)
	{
		_robot->addText("Limited by accel/cruise");
		_worldVel = newVel / newSpeed * maxSpeed;
	} else if (newSpeed < minSpeed)
	{
		_robot->addText("Limited by decel");
		_worldVel = newVel / newSpeed * minSpeed;
	} else {
		_robot->addText("Unchanged");
		_worldVel = newVel;
	}
	
	_robot->addText(QString().sprintf("pos %f %f", (double)_robot->config->translation.p, (double)_robot->config->translation.d));
	_lastPosError = posError;
}

void MotionControl::anglePD()
{
	_robot->addText(QString().sprintf("angle %f %f", (double)_robot->config->rotation.p, (double)_robot->config->rotation.d));
// 	_robot->state()->drawLine(_robot->pos, _robot->cmd.goalOrientation, Qt::black, "Motion");
	Point dir = (_robot->cmd.goalOrientation - _robot->pos).normalized();
	
	float error = Utils::fixAngleRadians(dir.angle() - _robot->angle * DegreesToRadians);
	
	_spin = error * _robot->config->rotation.p + (error - _lastAngleError) * _robot->config->rotation.d;
	_lastAngleError = error;
}

void MotionControl::run()
{
	positionPD();
	anglePD();
	
	// Scaling
	Point scaledVel = _worldVel * _robot->cmd.vScale;
	float scaledSpin = _spin * _robot->cmd.wScale;
	
	Point bodyVel = scaledVel.rotated(-_robot->angle);
	
	//FIXME - These are all 2011 numbers
	
	// The wheel angular velocity (rad/s) resulting from the largest wheel command
	//FIXME - Educated guess
	const float Max_Wheel_Speed = 140;
	
	// Effective wheel radius (m)
	const float Wheel_Radius = 0.026;
	
	// The largest wheel command that can be transmitted
	const int Max_Wheel_Command = 127;
	
	// Maximum linear speed without rotation (m/s)
	const float Max_Linear_Speed = Max_Wheel_Speed * Wheel_Radius;
	
	// Radius of circle containing the roller contact points (m)
// 	const float Contact_Circle_Radius = 0.0812;
	
	// Maximum angular speed without translation
// 	const float Max_Angular_Speed = Max_Linear_Speed / (Contact_Circle_Radius * 2 * M_PI);
	
	// Limit speed so we at least go in the right direction
	float s = bodyVel.mag();
	if (s > Max_Linear_Speed)
	{
		bodyVel = bodyVel / s * Max_Linear_Speed;
	}
	
	// Axle direction, pointing out of the robot
	const Point axles[4] =
	{
		Point(-1,  1).normalized(),
		Point( 1,  1).normalized(),
		Point( 1, -1).normalized(),
		Point(-1, -1).normalized()
	};
	
	// Set motor commands for linear motion
	int motors[4];
	int maxPos = 0;
	int maxNeg = 0;
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
		
		motors[i] = c;
	}
	
	// Get the largest motor command that was actually set
	maxPos = min(maxPos, Max_Wheel_Command);
	maxNeg = max(maxNeg, -Max_Wheel_Command);
	
	// Add as much rotation as we can without overflow
	int spinCommand = int(Max_Wheel_Command * scaledSpin / Max_Wheel_Speed + 0.5);
	
	if (spinCommand > 0 && (maxPos + spinCommand) > Max_Wheel_Command)
	{
		spinCommand = Max_Wheel_Command - maxPos;
	} else if (spinCommand < 0 && (maxNeg + spinCommand) < -Max_Wheel_Command)
	{
		spinCommand = -Max_Wheel_Command - maxNeg;
	}
	
	for (int i = 0; i < 4; ++i)
	{
		motors[i] += spinCommand;
	}
	
	// Store motor commands in the radio sub-packet
	for (int i = 0; i < 4; ++i)
	{
		_robot->radioTx.set_motors(i, motors[i]);
	}
}
