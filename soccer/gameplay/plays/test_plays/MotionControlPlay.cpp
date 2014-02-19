
#include "MotionControlPlay.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::MotionControlPlay, "Test")

Gameplay::Plays::MotionControlPlay::MotionControlPlay(GameplayModule *gameplay):
	Play(gameplay) {
		testStarted = false;
}

float Gameplay::Plays::MotionControlPlay::score(GameplayModule *gameplay) {
	return 0;
}

bool Gameplay::Plays::MotionControlPlay::run()
{
	//	path geometry
	float fudgeFactor = .15;
	float maxX = Field_Width / 2.0 - Robot_Radius - fudgeFactor;
	Point ptA(-maxX, 0.5);
	Point ptB(maxX, 0.5);

	//	get a robot (closest to ptA);
	OurRobot *robot = nullptr;
	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(robot, available, ptA);
	if (!robot) return true;

	//	move the bot toward ptA if we're not test
	if (!testStarted) {
		robot->move(ptA);
		if ( (robot->pos - ptA).mag() < 0.05 ) {
			robot->stop();
			testStarted = true;

			//	record start time.  we convert microseconds to seconds
			lapStartTime = timestamp();

			lastTime = lapStartTime;
		} else {
			return true;
		}
	}

	//	TODO: set lap start time
	//	TOD: reverse lap after each completion


	float totalDist = (ptA - ptB).mag();

	//	using trapezoidal velocity profile (m/s)
	float maxSpeed = 1.0;
	float maxAcceleration = 1.0;

	float pathDuration;	//	TODO

	//	when we're speeding up and slowing down - the sides of the trapezoid
	float rampTime = maxSpeed / maxAcceleration;
	float rampDist = 0.5 * maxAcceleration * powf(rampTime, 2.0);	//	Sf = 1/2*a*t^2

	//	when we're going at max speed
	float distAtMaxSpeed = (totalDist - 2.0 * rampDist);
	float timeAtMaxSpeed = distAtMaxSpeed / maxSpeed;

	//	how long we've been on this lap
	float timeIntoLap = (float)((timestamp() - lapStartTime) / 1000000.0f);
	float dt = lastTime - timeIntoLap;
	float velocityError = robot->vel.x - lastVelocityCommand *0.5;

	float targetX;
	float targetSpeed;
	float outputSpeed;
	if (timeIntoLap < rampTime) {	//	we're speeding up
		targetX = 0.5 * maxAcceleration * timeIntoLap * timeIntoLap;
		targetSpeed = maxAcceleration * timeIntoLap;

		robot->addText(QString("%1").arg(timeIntoLap));
	} else if (timeIntoLap < (rampTime + timeAtMaxSpeed)) {	//	at plateau, going max speed
		targetX = rampDist + maxSpeed * (timeIntoLap - rampTime);
		targetSpeed = maxSpeed;
		robot->addText("Plateau");
	} else {	//	we're slowing down
		float deccelTime = timeIntoLap - (rampTime + timeAtMaxSpeed);
		targetX = rampDist + distAtMaxSpeed + 
					maxSpeed * deccelTime - 0.5 * maxAcceleration * deccelTime * deccelTime;
		targetSpeed = maxSpeed - deccelTime * maxAcceleration;

		robot->addText("Ramp down");
	}
	robot->addText(QString("%1").arg(timeIntoLap));
	//	what the robot SHOULD be doing right now at time t = @timeIntoLap
	Point targetPos(ptA.x + targetX, ptA.y);
	Point targetVel(targetSpeed, 0);

	//	errorz
	Point posError = targetPos - robot->pos;
	Point velError = targetVel - robot->vel;




	float correctedVelocity = targetSpeed + posError.mag() * .2;

	outputSpeed = velocityError + correctedVelocity;
	lastVelocityCommand = outputSpeed;
	//Point vel(outputSpeed, 0);
	Point vel(targetSpeed, 0);
	//	set the robot's velocity
	robot->worldVelocity(vel);


	lastTime = timeIntoLap;
	return true;
}
