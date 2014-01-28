#pragma once

#include <Geometry2d/Point.hpp>

class OurRobot;

/**
 * This class handles the computer-side motion control for the robots.
 * It is responsible for most of what gets sent out in a RadioTx packet.
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
	void positionPD();
	void positionTrapezoidal();
	void anglePD();
	
	OurRobot *_robot;
	uint64_t _lastFrameTime;
	Geometry2d::Point _lastPos;
	float _lastAngle;
	Geometry2d::Point _lastPosError;
	float _lastAngleError;
	Geometry2d::Point _lastBodyVel;
	float _lastAngularVel;
};
