#include "Capture.hpp"

#include <RobotConfig.hpp>
#include <Utils.hpp>

using namespace std;
using namespace Geometry2d;

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
	_dribble_Speed  = new ConfigDouble(cfg, "Capture/Dribbler Speed", 127);

	_approach_Threshold_Reverse = new ConfigDouble(cfg, "Capture/Approach Threshold Reverse", .18);
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
}

bool Gameplay::Behaviors::Capture::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	// The direction we're facing
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	
	uint64_t now = timestamp();
	
	// State changes
	Point toBall = (ball().pos - robot->pos).normalized();
	float ballDist = ball().pos.distTo(robot->pos);
	float err = dir.dot(toBall);
	float ballSpeed = ball().vel.mag();
	bool behindBall = ((target - robot->pos).dot(ball().pos - robot->pos) > 0);



	// Target positioning for robot to trap the ball - if moving,
	// stop ball first, otherwise
	Point targetApproachPoint = ball().pos - toBall * *_approach_Distance;
	Point approachPoint = targetApproachPoint;

	// pick target based on velocity
	if (ballSpeed > *_stationaryMaxSpeed)
	{
		float interceptTime = ballDist / *robot->config->trapTrans.velocity;
		Point trapApproachPoint = ball().pos + ball().vel * interceptTime; // TODO: check if accel term is necessary
		approachPoint = trapApproachPoint;
	}


	if (_state == State_Approach)
	{
		robot->addText(QString("err %1 %2").arg(err).arg(robot->pos.distTo(approachPoint)));
		robot->addText(QString("ball dist = %1").arg(ballDist));
		robot->addText(QString("robot radius = %1").arg(Robot_Radius));
		if (robot->hasBall())
		{
			_state = State_Done;
		} else if ((ballDist - *_approach_Distance) < *_approach_Threshold) {
			_state = State_Capture;
			_lastBallTime = now;
		}
	} else if (_state == State_Capture) {
		// _lastBallTime is the last time we did not have the ball
		if (!robot->hasBall())
		{
			_lastBallTime = now;
		}
		
		if (!behindBall || ballDist > *_approach_Threshold_Reverse)
		{
			_state = State_Approach;
		}
		
		if ((now - _lastBallTime) >= *_capture_Time_Threshold)
		{
			_state = State_Done;
			
			_ccw = dir.cross(target - robot->pos) > 0;
			_lastBallTime = now;
		}
	}
	
	state()->drawLine(ball().pos, target, Qt::red);
	
	// Driving
	if (_state == State_Approach)
	{
		robot->addText("Approach");
		robot->avoidBall(*_approach_Clearance);
		robot->move(ball().pos);
		robot->face(ball().pos);
	} else if (_state == State_Capture) {
		robot->addText("Capture");
		
		double speed = max(0.0, 1.0 - double(now - _lastBallTime) / double(*_capture_Time_Threshold * *_capture_Decel)) * *_capture_Speed;
		
		robot->dribble(*_dribble_Speed);
		robot->worldVelocity(toBall * speed);
		robot->face(ball().pos);
	} else {
		robot->addText("Done");
		robot->dribble(*_dribble_Speed);
		return false;
	}
	
	return true;
}
