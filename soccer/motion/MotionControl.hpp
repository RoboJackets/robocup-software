#pragma once

class OurRobot;

// This class is responsible for everything stored in a RadioTx::Robot.
// Each robot has one.
class MotionControl
{
public:
	MotionControl(OurRobot *robot);
	
	void run();
	
private:
	OurRobot *_robot;
};
