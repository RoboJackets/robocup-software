
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

//	NOTE: doesn't handle triangle case
bool trapezoid(float pathLength, float maxSpeed, float maxAcc, float timeIntoLap, float &distOut, float &speedOut) {
	//	when we're speeding up and slowing down - the sides of the trapezoid
	float rampTime = maxSpeed / maxAcc;
	float rampDist = 0.5 * maxAcc * powf(rampTime, 2.0);	//	Sf = 1/2*a*t^2

	//	when we're going at max speed
	float distAtMaxSpeed = (pathLength - 2.0 * rampDist);
	float timeAtMaxSpeed = distAtMaxSpeed / maxSpeed;

	if (timeIntoLap < rampTime) {	//	we're speeding up
		distOut = 0.5 * maxAcc * timeIntoLap * timeIntoLap;
		speedOut = maxAcc * timeIntoLap;
	} else if (timeIntoLap < (rampTime + timeAtMaxSpeed)) {	//	at plateau, going max speed
		distOut = rampDist + maxSpeed * (timeIntoLap - rampTime);
		speedOut = maxSpeed;
	} else if (timeIntoLap < timeAtMaxSpeed + rampTime*2) {	//	we're slowing down
		float deccelTime = timeIntoLap - (rampTime + timeAtMaxSpeed);
		distOut = rampDist + distAtMaxSpeed + 
					maxSpeed * deccelTime - 0.5 * maxAcc * deccelTime * deccelTime;
		speedOut = maxSpeed - deccelTime * maxAcc;
	} else {
		//	restart for another lap
		return false;
	}

	return true;
}


Gameplay::Plays::MotionControlPlay::MotionControlPlay(GameplayModule *gameplay):
	Play(gameplay), _pidControllerX(1, 0, 0), _pidControllerY(1, 0, 0) {
		testStarted = false;

#if 0
		path = [](float timeIntoLap, Point &targetPos, Point &targetVel) {
			//	path geometry
			float fudgeFactor = .15;
			float maxX = Field_Width / 2.0 - Robot_Radius - fudgeFactor;
			Point ptA(-maxX, 0.5);
			Point ptB(maxX, 0.5);

			float totalDist = (ptA - ptB).mag();

			float pos, vel;
			bool notDone = trapezoid(
				totalDist,		//	length of path
				1.0,			//	max speed
				1.0,			//	max acc
				timeIntoLap,	//	time
				pos,			//	pos out
				vel 			//	vel out
				);

			//	what the robot SHOULD be doing right now at time t = @timeIntoLap
			targetPos = Point(ptA.x + pos, ptA.y);
			targetVel = Point(vel, 0);

			return notDone;
		};

#else
		//	circular path
		path = [](float timeIntoLap, Point &targetPos, Point &targetVel) {
			const float r = 0.5;
			const Point center(0, Field_Length / 2.0);
			const float circumference = 2.0*M_PI*r;

			float dist, vel;
			bool notDone = trapezoid(
				circumference,	//	length of path
				1.0,			//	max speed
				1.0,			//	max acc
				timeIntoLap,	//	time
				dist,			//	dist
				vel				//	vel
				);

			float angle = dist / circumference * 2.0 * M_PI;

			targetPos = Point(r, 0);
			targetPos.rotate(RadiansToDegrees*angle);
			targetPos += center;

			targetVel = Point(0, vel);
			targetVel.rotate(RadiansToDegrees*angle);

			return notDone;
		};
#endif
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
	// Point posError = targetPos - robot->pos;
	Point velError = targetVel - robot->vel;

	//	pid config
	_pidControllerX.kp = *_pid_p;
	_pidControllerX.ki = *_pid_i;
	_pidControllerX.kd = *_pid_d;
	_pidControllerY.kp = *_pid_p;
	_pidControllerY.ki = *_pid_i;
	_pidControllerY.kd = *_pid_d;

	//	controller
	Point correctedVelocity(
		targetVel.x + _pidControllerX.run(velError.x),
		targetVel.y + _pidControllerY.run(velError.y)
		);
		 // = targetVel + _pidController.run(velError.mag())*velError / velError.mag();
	robot->worldVelocity(correctedVelocity);

	return true;
}
