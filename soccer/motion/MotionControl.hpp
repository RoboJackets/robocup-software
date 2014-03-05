#pragma once

#include <Configuration.hpp>
#include <Geometry2d/Point.hpp>
#include <Pid.hpp>

class OurRobot;

/**
 * @brief Handles computer-side motion control
 * @details It is responsible for most of what gets sent out in a RadioTx packet.
 */
class MotionControl
{
public:
	MotionControl(OurRobot *robot);
	
	/**
	 * Leaves the motion values in the robot's radioTx at zero so, when sent,
	 * it will tell the robot to stop.
	 */
	void stopped() {}
	
	/**
	 * This runs PID control on the position and angle of the robot and
	 * sets values in the robot's radioTx packet.
	 */
	void run();
	

	static void createConfiguration(Configuration *cfg);

private:
	OurRobot *_robot;

	Pid _positionXController;
	Pid _positionYController;
	Pid _angleController;

	static ConfigDouble *_pid_pos_p;
	static ConfigDouble *_pid_pos_i;
	static ConfigDouble *_pid_pos_d;
	static ConfigDouble *_vel_mult;

	static ConfigDouble *_pid_angle_p;
	static ConfigDouble *_pid_angle_i;
	static ConfigDouble *_pid_angle_d;
	static ConfigDouble *_angle_vel_mult;
};
