#include "MotionControl.hpp"
#include <framework/RobotConfig.hpp>
#include <Robot.hpp>

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

void MotionControl::run()
{
	Point posError = _robot->cmd.goalPosition - _robot->pos;

#if 0
	Point worldVel;
	Point posErrorDir = posError.normalized();
	
	//FIXME - The velocity controller assumes that the robot is capable of infinite acceleration,
	// and thus the speed command will be executed completely (+noise) by the next frame.
	
	// Trapezoidal control
	const RobotConfig::Dynamics &config = _robot->config->motion.deg0;
	float curSpeed = _robot->vel.mag();
	
	// Distance left to travel to reach goal
	float distanceLeft = posError.mag();
	
	// How far would we travel if we decelerated at the limit?
	float minStoppingTime = curSpeed / config.deceleration;
	float minStoppingDistance = curSpeed * minStoppingTime + 0.5 * config.deceleration * minStoppingTime * minStoppingTime;
	
	//FIXME - Frame time from Processor
	const float FrameTime = 1.0 / 60.0;
	
	float targetSpeed;
	
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
		targetSpeed = min((double)config.velocity, curSpeed + config.acceleration * FrameTime);
		_robot->addText(QString("accel to %1").arg(targetSpeed));
	} else {
		// Constant velocity
		targetSpeed = config.velocity;
		_robot->addText(QString("cruise at %1").arg(targetSpeed));
	}
	worldVel = posErrorDir * targetSpeed;
#else
	Point worldVel = posError * _robot->config->motion.angle.p + (posError - _lastError) * _robot->config->motion.angle.d;
	_lastError = posError;
#endif
	
	Point bodyVel = worldVel.rotated(-_robot->angle);
	
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
	const float Contact_Circle_Radius = 0.0812;
	
	// Maximum angular speed without translation
	const float Max_Angular_Speed = Max_Linear_Speed / (Contact_Circle_Radius * 2 * M_PI);
	
#if 0
	// Locations of wheel contact points relative to robot center (forward=+X)
	const Point contactPoints[4] =
	{
		Point(Wheel_Radius * cos(M_PI * 3 / 4), Wheel_Radius * sin(M_PI * 3 / 4)),
		Point(Wheel_Radius * cos(M_PI * 1 / 4), Wheel_Radius * sin(M_PI * 1 / 4)),
		Point(Wheel_Radius * cos(M_PI * 7 / 4), Wheel_Radius * sin(M_PI * 7 / 4)),
		Point(Wheel_Radius * cos(M_PI * 5 / 4), Wheel_Radius * sin(M_PI * 5 / 4))
	};
#endif
	
	// Axle direction, pointing out of the robot
	const Point axles[4] =
	{
		Point(-1,  1).normalized(),
		Point( 1,  1).normalized(),
		Point( 1, -1).normalized(),
		Point(-1, -1).normalized()
	};
	
	float spin = 0;//Max_Angular_Speed * _robot->cmd.wScale;
	
	// Set motor commands for linear motion
	int motors[4];
	int maxMotor = 0;
	for (int i = 0; i < 4; ++i)
	{
		float w = axles[i].dot(bodyVel) / Wheel_Radius;
		int c = roundf(Max_Wheel_Command * w / Max_Wheel_Speed);
		
		// Keep track of the largest motor command before clipping
		maxMotor = max(maxMotor, abs(c));
		
		// Clipping
		c = max(-Max_Wheel_Command, c);
		c = min(Max_Wheel_Command, c);
		
		motors[i] = c;
	}
	
	if (maxMotor > Max_Wheel_Command)
	{
		printf("Motion clipped: %3d\n", maxMotor);
	}
	
	// Get the largest motor command that was actually set
	maxMotor = min(maxMotor, Max_Wheel_Command);
	
	// Add as much rotation as we can without overflow
	int spinCommand = int(Max_Wheel_Command * spin / Max_Wheel_Speed + 0.5);
	spinCommand = max(0, min(spinCommand, Max_Wheel_Command - maxMotor));
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
