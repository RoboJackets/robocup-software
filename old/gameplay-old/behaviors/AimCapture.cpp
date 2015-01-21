
#include "AimCapture.hpp"

#include <RobotConfig.hpp>
#include <Utils.hpp>

using namespace std;
using namespace Geometry2d;



#define BALL_MASS .15


namespace Gameplay
{
	namespace Behaviors
	{
		REGISTER_CONFIGURABLE(AimCapture)
	}
}

ConfigDouble *Gameplay::Behaviors::AimCapture::_stationaryMaxSpeed;
ConfigDouble *Gameplay::Behaviors::AimCapture::_approach_Distance;
ConfigDouble *Gameplay::Behaviors::AimCapture::_approach_Clearance;
ConfigDouble *Gameplay::Behaviors::AimCapture::_approach_Threshold;
ConfigDouble *Gameplay::Behaviors::AimCapture::_approach_Threshold_Reverse;
ConfigDouble *Gameplay::Behaviors::AimCapture::_capture_Speed;
ConfigDouble *Gameplay::Behaviors::AimCapture::_capture_Time_Threshold;
ConfigDouble *Gameplay::Behaviors::AimCapture::_capture_Decel;
ConfigDouble *Gameplay::Behaviors::AimCapture::_has_Ball_Dist;
ConfigDouble *Gameplay::Behaviors::AimCapture::_pivot_Speed;
ConfigDouble *Gameplay::Behaviors::AimCapture::_dribble_Speed;

ConfigDouble *Gameplay::Behaviors::AimCapture::_capture_perp_p;
ConfigDouble *Gameplay::Behaviors::AimCapture::_capture_parallel_p;


ConfigDouble *Gameplay::Behaviors::AimCapture::_intercept_velocity_min;

void Gameplay::Behaviors::AimCapture::createConfiguration(Configuration* cfg)
{
	_stationaryMaxSpeed = new ConfigDouble(cfg, "AimCapture/Ball Speed Threshold", 0.5);
	_approach_Distance = new ConfigDouble(cfg, "AimCapture/Approach Distance", 0.1);
	_approach_Clearance  = new ConfigDouble(cfg, "AimCapture/Approach Clearance", 0.05);
	_approach_Threshold  = new ConfigDouble(cfg, "AimCapture/Approach Threshold", 0.13);
	_capture_Speed  = new ConfigDouble(cfg, "AimCapture/AimCapture Speed", 0.3);
	_capture_Time_Threshold  = new ConfigDouble(cfg, "AimCapture/AimCapture Time Threshold", 300 * 1000);
	_capture_Decel  = new ConfigDouble(cfg, "AimCapture/AimCapture Decel", 0.8);
	_has_Ball_Dist  = new ConfigDouble(cfg, "AimCapture/Has Ball Distance", 0.1);
	_pivot_Speed  = new ConfigDouble(cfg, "AimCapture/Pivot Speed", 0.5 * M_PI);
	_dribble_Speed  = new ConfigDouble(cfg, "AimCapture/Dribbler Speed", 127);

	_approach_Threshold_Reverse = new ConfigDouble(cfg, "AimCapture/Approach Threshold Reverse", .18);

	_intercept_velocity_min = new ConfigDouble(cfg, "AimCapture/Intercept Vel Min", .5);

	_capture_perp_p = new ConfigDouble(cfg, "AimCapture/Capture Perp K_P", 3);
	_capture_parallel_p = new ConfigDouble(cfg, "AimCapture/Capture Parallel K_P", 2);
}

Gameplay::Behaviors::AimCapture::AimCapture(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	restart();
	
	target = Point(0, Field_Length); // center of goal
}

void Gameplay::Behaviors::AimCapture::restart()
{
	_state = State_Approach;
	enable_pivot = true;
	_captureDist = *_approach_Distance;
}


float Gameplay::Behaviors::AimCapture::estimatedKickSpeed() {
	return 7;
}

Point Gameplay::Behaviors::AimCapture::calculateBallPseudoFinalMomentum(Point &t) {
	Point dir = t - ball().pos;

	return dir * estimatedKickSpeed() * BALL_MASS;
}






Point Gameplay::Behaviors::AimCapture::interceptTarget() {
	Point receiveTarget;  // Point at which to place robot mouth
	Point passVector;     // Direction of pass

	Line passLine(ball().pos, ball().pos + ball().vel);
	receiveTarget = passLine.nearestPoint(robot->pos);
	// passVector = ball().vel;

	return receiveTarget;
}

bool Gameplay::Behaviors::AimCapture::canIntercept() {
	Point toBall = robot->pos - ball().pos;
	bool ballTowardsUs = ball().vel.dot(toBall) > 0;

	return ballTowardsUs && (ball().vel.mag() > *_intercept_velocity_min);
}


Point Gameplay::Behaviors::AimCapture::lineUpTarget() {

	// Point approachDir = 

	Point initialApproachTarget = ball().pos - approachDir * *_approach_Distance;
}

bool Gameplay::Behaviors::AimCapture::canApproach() {
	
	Segment(robot->pos, )
}




bool Gameplay::Behaviors::AimCapture::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	

	//	FIXME: lag term?


	Point impulse = calculateBallPseudoFinalMomentum(target) - (ball().vel * BALL_MASS);

	Point approachDir = impulse.normalized();	//	direction from robot to ball when we're approaching



	// //	if we're far away, reset _captureDist
	// if ( robot->pos.distTo(ball().pos) > .3 ) {	//	FIXME: ConfigDouble
	// 	_captureDist = *_approach_Distance;
	// }



	float ballDist = robot->pos.distTo(ball().pos);



	bool closeToBall = ballDist < *_approach_Distance;
	bool fairlyCloseToBall = ballDist < *_approach_Distance * 1.3;	//	TODO: constant?

	
	bool behindBall = ((target - robot->pos).dot(ball().pos - robot->pos) > 0);



	Line approachLine(ball().pos - approachDir, ball().pos);


	Point initialApproachTarget = ball().pos - approachDir * *_approach_Distance;
	state()->drawCircle(initialApproachTarget, .02, Qt::blue);


	Point closestApproachLinePoint = approachLine.nearestPoint(robot->pos);


	Point approachDiff = closestApproachLinePoint - robot->pos;

	float perpApproachError = approachDiff.mag() * (approachDiff.dot(approachDir) > 0 ? 1 : -1);
	robot->addText(QString("Perp err %1").arg(perpApproachError));
	// robot->addText(QString("err %1 %2").arg(err).arg(robot->pos.distTo(approachPoint)));


	float parallelApproachError = (initialApproachTarget - closestApproachLinePoint).mag();




	bool closeToApproachPoint = robot->pos.distTo(initialApproachTarget);



	//	state changes
	if ( _state == State_Done && !robot->hasBall() ) {
		_state = State_Approach;
	} else if ( _state == State_Approach ) {
		if ( behindBall && closeToApproachPoint ) {
			_state = State_Capture;
		}
	} else if ( _state == State_Capture ) {
		if ( !fairlyCloseToBall || !behindBall || std::abs(perpApproachError) > Robot_Radius ) {
			_state = State_Approach;
		}
	}


	//	avoid ball while we're approaching
	if ( _state == State_Approach ) {
		robot->avoidBall(*_approach_Clearance);
	} else {
		robot->disableAvoidBall();
	}

	//	only dribble when we're close to the ball
	if ( _state == State_Capture ) {
		robot->dribble(*_dribble_Speed);
	} else {
		robot->dribble(0);
	}





	





	if ( _state == State_Capture ) {

		_perpindicularPidController.kp = *_capture_perp_p;	//	FIXME: ?

		Point perpCorrectionDir = Point(closestApproachLinePoint - robot->pos).normalized();
		Point perpErrorCorrectionVel = perpApproachError * _perpindicularPidController.run(perpApproachError) * perpCorrectionDir;


		_parallelPidController.kp = *_capture_parallel_p;	//	FIXME: ?

		float approachSpeed = *_capture_Speed;
		Point approachVel = approachSpeed * approachDir;


		Point botVel = ball().vel + approachVel + perpErrorCorrectionVel;
		robot->addText(QString("botVel(%1, %2)").arg(botVel.x).arg(botVel.y));

		robot->worldVelocity(botVel);
	} else if ( _state == State_Approach ) {
		robot->move(initialApproachTarget);
	}
	


	//	Robot Facing
	Point faceTarget;
	if ( _state == State_Capture && !robot->hasBall() ) {
		faceTarget = ball().pos;
	} else {
		faceTarget = robot->pos + (approachDir * 100);
	}


	//	draw the approach direction ray from the bot
	state()->drawLine(robot->pos, faceTarget, Qt::red);
	robot->face(faceTarget);



	//	detect ball hold transitions (!ball -> ball OR ball -> !ball)
	if ( _hadBall || !robot->hasBall() ) {
		_hadBall = false;
	} else if ( !_hadBall && robot->hasBall() ) {
		_hadBall = true;
		_ballHoldStartTime = timestamp();
	}

	//	see if we've had the ball for long enough to consider it captured
	// if ( _hadBall ) {
	// 	Time ballHoldDuration = timestamp() - _ballHoldStartTime;
	// 	if ( ballHoldDuration > 20 ) { //*_capture_Time_Threshold ) {
	// 		_state = State_Done;
	// 		return false;
	// 	}
	// }


	if ( robot->hasBall() ) {
		_state = State_Done;
		return false;
	}



	if ( _state == State_Approach ) {
		robot->addText(QString("AC: Approach"));
	} else if ( _state == State_Capture ) {
		robot->addText(QString("AC: Capture"));
	} else {
		robot->addText(QString("AC: Done"));
	}




	return true;
}
