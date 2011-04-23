#include "MotionControl.hpp"

using namespace Geometry2d;

MotionControl::MotionControl(OurRobot *robot)
{
	_robot = robot;

	_robot->radioTx.set_board_id(_robot->shell);
	for (int m = 0; m < 4; ++m)
	{
		_robot->radioTx.add_motors(0);
	}
}

void MotionControl::run()
{
	Point bodyVel;
	float spin = 0;
	
	// The angular velocity (rad/s) resulting from the largest wheel command
	//FIXME - Educated guess
	const float Max_Wheel_Speed = 140;
	
	// The largest wheel command that can be transmitted
	const int Max_Wheel_Command = 127;
	
	//FIXME - Real numbers, or prove that it doesn't matter
	const Point axles[4] =
	{
		Point(-1, -1),
		Point(-1, 1),
		Point(1, 1),
		Point(1, -1),
	};
	
	// Set motor commands for linear motion
	bool saturated = false;
	for (int m = 0; m < 4; ++m)
	{
		float w = axles[i].dot(bodyVel);
		int c = int(Max_Wheel_Command * w / Max_Wheel_Speed + 0.5);
		if (c < -Max_Wheel_Command)
		{
			c = -Max_Wheel_Command;
			saturated = true;
		} else if (c > Max_Wheel_Command)
		{
			c = Max_Wheel_Command;
			saturated = true;
		}
		_robot->radioTx.set_motors(i, c);
	}
	
	// Add motor commands for rotation
	for (int m = 0; m < 4; ++m)
	{
	}
}
