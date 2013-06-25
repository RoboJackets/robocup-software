#include "Capture.hpp"

#include <framework/RobotConfig.hpp>
#include <Utils.hpp>

using namespace std;
using namespace Geometry2d;



#define BALL_MASS .15


namespace Gameplay
{
	namespace Behaviors
	{
		REGISTER_CONFIGURABLE(Capture)
	}
}

ConfigDouble *Gameplay::Behaviors::Capture::_stationaryMaxSpeed;
ConfigDouble *Gameplay::Behaviors::Capture::_approach_Distance;
ConfigDouble *Gameplay::Behaviors::Capture::_approach_Clearance;
ConfigDouble *Gameplay::Behaviors::Capture::_approach_Threshold;
ConfigDouble *Gameplay::Behaviors::Capture::_approach_Threshold_Reverse;
ConfigDouble *Gameplay::Behaviors::Capture::_capture_Speed;
ConfigDouble *Gameplay::Behaviors::Capture::_capture_Time_Threshold;
ConfigDouble *Gameplay::Behaviors::Capture::_capture_Decel;
ConfigDouble *Gameplay::Behaviors::Capture::_has_Ball_Dist;
ConfigDouble *Gameplay::Behaviors::Capture::_pivot_Speed;
ConfigDouble *Gameplay::Behaviors::Capture::_dribble_Speed;

void Gameplay::Behaviors::Capture::createConfiguration(Configuration* cfg)
{
	_stationaryMaxSpeed = new ConfigDouble(cfg, "Capture/Ball Speed Threshold", 0.5);
	_approach_Distance = new ConfigDouble(cfg, "Capture/Approach Distance", 0.1);
	_approach_Clearance  = new ConfigDouble(cfg, "Capture/Approach Clearance", 0.05);
	_approach_Threshold  = new ConfigDouble(cfg, "Capture/Approach Threshold", 0.13);
	_capture_Speed  = new ConfigDouble(cfg, "Capture/Capture Speed", 0.3);
	_capture_Time_Threshold  = new ConfigDouble(cfg, "Capture/Capture Time Threshold", 300 * 1000);
	_capture_Decel  = new ConfigDouble(cfg, "Capture/Capture Decel", 0.8);
	_has_Ball_Dist  = new ConfigDouble(cfg, "Capture/Has Ball Distance", 0.1);
	_pivot_Speed  = new ConfigDouble(cfg, "Capture/Pivot Speed", 0.5 * M_PI);
	_dribble_Speed  = new ConfigDouble(cfg, "Capture/Dribbler Speed", 127);

	_approach_Threshold_Reverse = new ConfigDouble(cfg, "Capture/Approach Threshold Reverse", .18);

	_perp_approach_error_threshold = new ConfigDouble(cfg, "Capture/Perp Approach Error Threshold", .04);
}

Gameplay::Behaviors::Capture::Capture(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	restart();
	
	target = Point(0, Field_Length); // center of goal
}

void Gameplay::Behaviors::Capture::restart()
{
	_state = State_Approach;
	_ccw = true;
	enable_pivot = true;
	_captureDist = *_approach_Distance;
}


float Gameplay::Behaviors::Capture::estimatedKickSpeed() {
	return 7;
}

Point Gameplay::Behaviors::Capture::calculateBallPseudoFinalMomentum(Point &t) {
	Point dir = t - ball().pos;

	return dir * estimatedKickSpeed() * BALL_MASS;
}


bool Gameplay::Behaviors::Capture::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	

	//	FIXME: lag term?


	Point impulse = calculateBallPseudoFinalMomentum(target) - (ball().vel * BALL_MASS);

	Point approachDir = impulse.normalized();	//	direction from robot to ball when we're approaching



	//	if we're far away, reset _captureDist
	if ( robot->pos.distTo(ball().pos) > .3 ) {	//	FIXME: ConfigDouble
		_captureDist = *_approach_Distance;
	}




	Point approachTarget = ball().pos - approachDir * _captureDist;


	robot->move(approachTarget);

	state()->drawCircle(approachTarget, .02, Qt::blue);



	Line approachLine(ball().pos - approachDir, ball().pos);
	Point closestApproachLinePoint = approachLine.nearestPoint(robot->pos);


	float perpApproachError = approachLine.distTo(robot->pos);
	float parallelApproachError = (approachTarget - closestApproachLinePoint).mag();

	//	FIXME: based on the errors, adjust approachTarget


	//	adjust _captureDist based on 
	if ( perpApproachError < *_perp_approach_error_threshold ) {
		bool passedApproachTarget = parallelApproachError < 0.02;

		if ( passedApproachTarget )
			float dist = closestApproachLinePoint.distTo(ball().pos);
			_captureDist = dist;
		}
	}




	





	//	avoid ball if we're not behind it
	bool behindBall = ((target - robot->pos).dot(ball().pos - robot->pos) > 0);
	if ( behindBall ) {
		robot->disableAvoidBall();
	} else {
		robot->avoidBall(*_approach_Clearance);
	}










	bool finalApproach = _captureDist < *_approach_Distance;

	//	only dribble when we're close to the ball
	if ( finalApproach ) {
		robot->dribble(*_dribble_Speed);
	} else {
		robot->dribble(0);
	}



	//	Robot Facing
	Point faceTarget;
	if ( finalApproach || robot->hasBall() ) {
		faceTarget = robot->pos + (approachDir * 100);
	} else {
		faceTarget = ball().pos;
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
	if ( _hadBall ) {
		uint64_t ballHoldDuration = timestamp() - _ballHoldStartTime;
		if ( ballHoldDuration > *_capture_Time_Threshold ) {
			_state = State_Done;
			return false;
		}
	}


	return true;


	//======================================================================================





	// // The direction we're facing
	// const Point dir = Point::direction(robot->angle * DegreesToRadians);
	
	// uint64_t now = timestamp();
	
	// // State changes
	// Point toBall = (ball().pos - robot->pos).normalized();
	// float err = dir.dot(toBall);
	// float ballSpeed = ball().vel.mag();

	// // Target positioning for robot to trap the ball - if moving,
	// // stop ball first, otherwise
	// Point targetApproachPoint = ball().pos - (target - ball().pos).normalized() * *_approach_Distance;
	// Point approachPoint = targetApproachPoint;

	// // pick target based on velocity
	// if (ballSpeed > *_stationaryMaxSpeed)
	// {
	// 	float interceptTime = ballDist / *robot->config->trapTrans.velocity;
	// 	Point trapApproachPoint = ball().pos + ball().vel * interceptTime; // TODO: check if accel term is necessary
	// 	approachPoint = trapApproachPoint;
	// }////////////////////////////////////////////////////////////////////////

	// if (_state == State_Approach)
	// {
	// 	robot->addText(QString("err %1 %2").arg(err).arg(robot->pos.distTo(approachPoint)));
	// 	if (robot->hasBall())
	// 	{
	// 		if (enable_pivot)
	// 		{
	// 			_state = State_Pivoting;
	// 			_ccw = ((target - ball().pos).cross(target - ball().pos) > 0);
	// 		} else
	// 		{
	// 			_state = State_Done;
	// 		}
	// 	} else if (robot->pos.nearPoint(approachPoint, *_approach_Threshold) && err >= cos(10 * DegreesToRadians))
	// 	{
	// 		_state = State_Capture;
	// 		_lastBallTime = now;
	// 	}
	// } else if (_state == State_Capture)
	// {
	// 	// _lastBallTime is the last time we did not have the ball
	// 	if (!robot->hasBall())
	// 	{
	// 		_lastBallTime = now;
	// 	}
		
	// 	if (!behindBall || ballDist > *_approach_Threshold_Reverse)
	// 	{
	// 		_state = State_Approach;
	// 	}
		
	// 	if ((now - _lastBallTime) >= *_capture_Time_Threshold)
	// 	{
	// 		if (!enable_pivot || (ball().pos.nearPoint(robot->pos, *_has_Ball_Dist) && err >= cos(20 * DegreesToRadians)))
	// 		{
	// 			_state = State_Done;
	// 		} else {
	// 			_state = State_Pivoting;
	// 		}
	// 		_ccw = dir.cross(target - robot->pos) > 0;
	// 		_lastBallTime = now;
	// 	}
	// } else if (_state == State_Pivoting)
	// {
	// 	if (!enable_pivot)
	// 	{
	// 		_state = State_Done;
	// 	}

	// 	// _lastBallTime is the last time we had the ball
	// 	if (robot->hasBall())
	// 	{
	// 		_lastBallTime = now;
	// 	}

	// 	if ((!robot->hasBall() && (state()->timestamp - _lastBallTime) > 500000) || !ball().pos.nearPoint(robot->pos, *_approach_Distance))
	// 	{
	// 		_state = State_Approach;
	// 	} else if (ball().pos.nearPoint(robot->pos, *_has_Ball_Dist) && err >= cos(20 * DegreesToRadians))
	// 	{
	// 		_state = State_Done;
	// 	}
	// }
	
	// state()->drawLine(ball().pos, target, Qt::red);
	
	// // Driving
	// if (_state == State_Approach)
	// {
	// 	robot->addText("Approach");
	// 	robot->avoidBall(*_approach_Clearance);
	// 	robot->move(approachPoint);
	// 	robot->face(ball().pos);
	// } else if (_state == State_Capture)
	// {
	// 	robot->addText("Capture");
		
	// 	double speed = max(0.0, 1.0 - double(now - _lastBallTime) / double(*_capture_Time_Threshold * *_capture_Decel)) * *_capture_Speed;
		
	// 	robot->dribble(*_dribble_Speed);
	// 	robot->worldVelocity(toBall * speed);
	// 	robot->face((ball().pos - robot->pos) * 1.2 + robot->pos);
	// } else if (_state == State_Pivoting)
	// {
	// 	robot->addText("Pivoting");
	// 	state()->drawLine(robot->pos, robot->pos + dir * 8, Qt::white);
	// 	state()->drawLine(ball().pos, target, Qt::yellow);
	// 	state()->drawLine(robot->pos, (ball().pos - robot->pos).normalized() * 8, Qt::green);

	// 	// See if we've gotten close enough
	// 	float error = dir.dot((target - ball().pos).normalized());

	// 	// Decide which direction to rotate around the ball
	// 	Point rb = ball().pos - robot->pos;
	// 	if (rb.cross(target - ball().pos) > 0)
	// 	{
	// 		_ccw = true;
	// 	} else if ((target - ball().pos).cross(rb) > 0)
	// 	{
	// 		_ccw = false;
	// 	}
	// 	robot->addText(QString("Pivot %1 %2").arg(
	// 		QString::number(acos(error) * RadiansToDegrees),
	// 		QString::number(_ccw ? 1 : 0)));
		
	// 	robot->pivot(*_pivot_Speed * (_ccw ? 1 : -1), ball().pos);
	// 	robot->dribble(*_dribble_Speed);
	// } else {
	// 	robot->addText("Done");
	// 	robot->dribble(*_dribble_Speed);
	// 	return false;
	// }
	
	// return true;
}
