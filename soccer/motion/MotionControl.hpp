#pragma once

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
	
private:
	OurRobot *_robot;
	uint64_t _lastFrameTime;

	Pid _positionController;
	Pid _angleController;
};
