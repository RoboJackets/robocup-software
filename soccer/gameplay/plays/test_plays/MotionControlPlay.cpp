
#include "MotionControlPlay.hpp"

using namespace std;
using namespace Geometry2d;


REGISTER_PLAY_CATEGORY(Gameplay::Plays::MotionControlPlay, "Test")


Gameplay::Plays::MotionControlPlay::MotionControlPlay(GameplayModule *gameplay):
	Play(gameplay), _pidControllerX(1, 0, 0), _pidControllerY(1, 0, 0) {
		testStarted = false;

#if 1
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
				2.0,			//	max speed
				1.5,			//	max acc
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
			const Point center(0, Field_Length / 2.0 - r - 0.1);
			const float circumference = 2.0*M_PI*r;

			float dist, vel;
			bool notDone = trapezoid(
				circumference,	//	length of path
				0.75,			//	max speed
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

	//	query the path for where we should be
	Point targetPos, targetVel;
	if (!path(timeIntoLap, targetPos, targetVel)) {
		testStarted = false;
	}

	//	draw target pt
	state()->drawCircle(targetPos, .04, Qt::blue);

	//	pid config
	_pidControllerX.kp = *_pid_p;
	_pidControllerX.ki = *_pid_i;
	_pidControllerX.kd = *_pid_d;

	_pidControllerY.kp = *_pid_p;
	_pidControllerY.ki = *_pid_i;
	_pidControllerY.kd = *_pid_d;

	//	controller
	Point posError = targetPos - robot->pos;
	targetVel *= *_v_p;
	Point correctedVelocity(
		targetVel.x + _pidControllerX.run(posError.x),
		targetVel.y + _pidControllerY.run(posError.y)
		);

	robot->worldVelocity(correctedVelocity);

	return true;
}
