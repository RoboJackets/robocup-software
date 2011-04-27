#pragma once

#include <Geometry2d/Point.hpp>

class OurRobot;

// This class is responsible for everything stored in a RadioTx::Robot.
// Each robot has one.
class MotionControl
{
public:
	MotionControl(OurRobot *robot);
	
	void stopped();
	
	void run();
	
private:
	void positionPD();
	void positionTrapezoidal();
	void anglePD();
	
	OurRobot *_robot;
	float _wheelVel[4];
	float _out[4];
	float _integral[4];
	uint64_t _lastFrameTime;
	Geometry2d::Point _lastPos;
	float _lastAngle;
	Geometry2d::Point _velocity;
	float _angularVelocity;
	Geometry2d::Point _lastPosError;
	float _lastAngleError;
	Geometry2d::Point _worldVel;
	float _spin;
};
