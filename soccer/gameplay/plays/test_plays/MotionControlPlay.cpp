
#include "MotionControlPlay.hpp"

using namespace std;
using namespace Geometry2d;


namespace Gameplay {
	namespace Plays {
		REGISTER_CONFIGURABLE(MotionControlPlay);
	}
}

ConfigDouble *Gameplay::Plays::MotionControlPlay::_pid_p;
ConfigDouble *Gameplay::Plays::MotionControlPlay::_pid_i;
ConfigDouble *Gameplay::Plays::MotionControlPlay::_pid_d;

void Gameplay::Plays::MotionControlPlay::createConfiguration(Configuration *cfg) {
	_pid_p = new ConfigDouble(cfg, "MotionControlPlay/pid_p", 6.5);
	_pid_i = new ConfigDouble(cfg, "MotionControlPlay/pid_i", 0.0001);
	_pid_d = new ConfigDouble(cfg, "MotionControlPlay/pid_d", 2);
}

REGISTER_PLAY_CATEGORY(Gameplay::Plays::MotionControlPlay, "Test")

Gameplay::Plays::MotionControlPlay::MotionControlPlay(GameplayModule *gameplay):
	Play(gameplay), _pidController(1, 0, 0) {
		testStarted = false;

		path = [](float timeIntoLap, Point &targetPos, Point &targetVel) {
			//	path geometry
			float fudgeFactor = .15;
			float maxX = Field_Width / 2.0 - Robot_Radius - fudgeFactor;
			Point ptA(-maxX, 0.5);
			Point ptB(maxX, 0.5);

			float totalDist = (ptA - ptB).mag();

			//	using trapezoidal velocity profile (m/s)
			float maxSpeed = 1.0;
			float maxAcceleration = 1.0;

			//	when we're speeding up and slowing down - the sides of the trapezoid
			float rampTime = maxSpeed / maxAcceleration;
			float rampDist = 0.5 * maxAcceleration * powf(rampTime, 2.0);	//	Sf = 1/2*a*t^2

			//	when we're going at max speed
			float distAtMaxSpeed = (totalDist - 2.0 * rampDist);
			float timeAtMaxSpeed = distAtMaxSpeed / maxSpeed;

			float targetX;
			float targetSpeed;
			if (timeIntoLap < rampTime) {	//	we're speeding up
				targetX = 0.5 * maxAcceleration * timeIntoLap * timeIntoLap;
				targetSpeed = maxAcceleration * timeIntoLap;

				// robot->addText(QString("%1").arg(timeIntoLap));
			} else if (timeIntoLap < (rampTime + timeAtMaxSpeed)) {	//	at plateau, going max speed
				targetX = rampDist + maxSpeed * (timeIntoLap - rampTime);
				targetSpeed = maxSpeed;
				// robot->addText("Plateau");
			} else if (timeIntoLap < timeAtMaxSpeed + rampTime*2) {	//	we're slowing down
				float deccelTime = timeIntoLap - (rampTime + timeAtMaxSpeed);
				targetX = rampDist + distAtMaxSpeed + 
							maxSpeed * deccelTime - 0.5 * maxAcceleration * deccelTime * deccelTime;
				targetSpeed = maxSpeed - deccelTime * maxAcceleration;

				// robot->addText("Ramp down");
			} else {
				//	restart for another lap
				return false;
			}
			// robot->addText(QString("%1").arg(timeIntoLap));
			//	what the robot SHOULD be doing right now at time t = @timeIntoLap
			targetPos = Point(ptA.x + targetX, ptA.y);
			targetVel = Point(targetSpeed, 0);

			return true;
		};
}

float Gameplay::Plays::MotionControlPlay::score(GameplayModule *gameplay) {
	return 0;
}

bool Gameplay::Plays::MotionControlPlay::run()
{
	//	get the start info
	Point startPt; Point startVel;
	path(0, startPt, startVel);

	//	get a robot (closest to startPt);
	OurRobot *robot = nullptr;
	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(robot, available, startPt);
	if (!robot) return true;

	//	move the bot toward startPt if we're not test
	if (!testStarted) {
		robot->move(startPt);
		if ( (robot->pos - startPt).mag() < 0.05 ) {
			robot->stop();
			testStarted = true;

			//	record start time.  we convert microseconds to seconds
			lapStartTime = timestamp();
		} else {
			return true;
		}
	}


	//	how long we've been on this lap
	float timeIntoLap = (float)((timestamp() - lapStartTime) / 1000000.0f);


	Point targetPos, targetVel;
	if (!path(timeIntoLap, targetPos, targetVel)) {
		testStarted = false;
	}

	//	draw
	state()->drawCircle(targetPos, .04, Qt::blue);
	// state()->drawLine(ptA, ptB, Qt::blue);

	//	errorz
	Point posError = targetPos - robot->pos;
	Point velError = targetVel - robot->vel;

	//	pid config
	_pidController.kp = *_pid_p;
	_pidController.ki = *_pid_i;
	_pidController.kd = *_pid_d;

	//	controller
	float correctedVelocity = targetVel.x + _pidController.run(velError.mag());
	Point vel(correctedVelocity, 0);
	//	set the robot's velocity
	robot->worldVelocity(vel);

	return true;
}
