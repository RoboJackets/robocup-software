#include "Capture.hpp"

#include <framework/RobotConfig.hpp>
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

// ConfigDouble *Gameplay::Behaviors::Capture::_stationaryMaxSpeed;
// ConfigDouble *Gameplay::Behaviors::Capture::_approach_Distance;
// ConfigDouble *Gameplay::Behaviors::Capture::_approach_Clearance;
// ConfigDouble *Gameplay::Behaviors::Capture::_approach_Threshold;
// ConfigDouble *Gameplay::Behaviors::Capture::_approach_Threshold_Reverse;
// ConfigDouble *Gameplay::Behaviors::Capture::_capture_Speed;
// ConfigDouble *Gameplay::Behaviors::Capture::_capture_Time_Threshold;
// ConfigDouble *Gameplay::Behaviors::Capture::_capture_Decel;
// ConfigDouble *Gameplay::Behaviors::Capture::_has_Ball_Dist;
// ConfigDouble *Gameplay::Behaviors::Capture::_pivot_Speed;
// ConfigDouble *Gameplay::Behaviors::Capture::_dribble_Speed;


ConfigDouble *Gameplay::Behaviors::Capture::_drive_around_dist;
ConfigDouble *Gameplay::Behaviors::Capture::_setup_to_charge_thresh;
ConfigDouble *Gameplay::Behaviors::Capture::_escape_charge_thresh;
ConfigDouble *Gameplay::Behaviors::Capture::_setup_ball_avoid;
ConfigDouble *Gameplay::Behaviors::Capture::_accel_bias;
ConfigDouble *Gameplay::Behaviors::Capture::_facing_thresh;
ConfigDouble *Gameplay::Behaviors::Capture::_max_speed;
ConfigDouble *Gameplay::Behaviors::Capture::_proj_time;
ConfigDouble *Gameplay::Behaviors::Capture::_dampening;
ConfigDouble *Gameplay::Behaviors::Capture::_done_thresh;

ConfigDouble *Gameplay::Behaviors::Capture::_dribble_speed;

void Gameplay::Behaviors::Capture::createConfiguration(Configuration* cfg)
{
	// _stationaryMaxSpeed = new ConfigDouble(cfg, "Capture/Ball Speed Threshold", 0.5);
	// _approach_Distance = new ConfigDouble(cfg, "Capture/Approach Distance", 0.1);
	// _approach_Clearance  = new ConfigDouble(cfg, "Capture/Approach Clearance", 0.05);
	// _approach_Threshold  = new ConfigDouble(cfg, "Capture/Approach Threshold", 0.13);
	// _capture_Speed  = new ConfigDouble(cfg, "Capture/Capture Speed", 0.3);
	// _capture_Time_Threshold  = new ConfigDouble(cfg, "Capture/Capture Time Threshold", 300 * 1000);
	// _capture_Decel  = new ConfigDouble(cfg, "Capture/Capture Decel", 0.8);
	// _has_Ball_Dist  = new ConfigDouble(cfg, "Capture/Has Ball Distance", 0.1);
	// _pivot_Speed  = new ConfigDouble(cfg, "Capture/Pivot Speed", 0.5 * M_PI);
	// _dribble_Speed  = new ConfigDouble(cfg, "Capture/Dribbler Speed", 127);



	_drive_around_dist = new ConfigDouble(cfg, "Capture/Drive Around Dist", 0.25);
	_setup_to_charge_thresh = new ConfigDouble(cfg, "Capture/Charge Thresh", 0.1);
	_escape_charge_thresh = new ConfigDouble(cfg, "Capture/Escape Charge Thresh", 0.1);
	_setup_ball_avoid = new ConfigDouble(cfg, "Capture/Setup Ball Avoid", Ball_Radius * 2.0);
	_accel_bias = new ConfigDouble(cfg, "Capture/Accel Bias", 0.1);
	_facing_thresh = new ConfigDouble(cfg, "Capture/Facing Thresh - Deg", 10);
	_max_speed = new ConfigDouble(cfg, "Capture/Max Charge Speed", 1.5);
	_proj_time = new ConfigDouble(cfg, "Capture/Ball Project Time", 0.4);
	_dampening = new ConfigDouble(cfg, "Capture/Ball Project Dampening", 0.8);
	_done_thresh = new ConfigDouble(cfg, "Capture/Done State Thresh", 0.11);

	_dribble_speed = new ConfigDouble(cfg, "Capture/Dribble Speed", 70);


	// _approach_Threshold_Reverse = new ConfigDouble(cfg, "Capture/Approach Threshold Reverse", .18);
}

Gameplay::Behaviors::Capture::Capture(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay)
{
	restart();
	
	target = Point(0, Field_Length); // center of goal

	use_dribbler = true;
}

void Gameplay::Behaviors::Capture::restart()
{
	// enable_pivot = true;

	_state = State_Approach;
	scaleAcc = 1.0;
	scaleSpeed = 1.0;
	scaleW = 1.0;
	ballClose = false;
}

bool Gameplay::Behaviors::Capture::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	


	// project the ball ahead to handle movement
	double dt = *_proj_time;
	Point ballPos = ball().pos + ball().vel * dt * _dampening->value();  // projecting
//	Point ballPos = ball().pos; // no projecting
	Line targetLine(ballPos, target);
	const Point dir = Point::direction(robot->angle * DegreesToRadians);
	double facing_thresh = cos(*_facing_thresh * DegreesToRadians);
	double facing_err = dir.dot((target - ballPos).normalized());
	

	if(ballPos.distTo(robot->pos) <= *_done_thresh)
	{
		ballClose = true;
	}

	// State changes
	if (_state == State_Approach)
	{
		if (targetLine.distTo(robot->pos) <= *_setup_to_charge_thresh &&
				targetLine.delta().dot(robot->pos - ballPos) <= -Robot_Radius &&
				facing_err >= facing_thresh &&
				robot->vel.mag() < 0.05)
		{
			_state = State_Capture;
		}
		

		//if the ball if further away than the back off distance for the setup stage
		if(ballClose && ballPos.distTo(robot->pos) > *_drive_around_dist + Robot_Radius)
		{
			_state = State_Done;
		}
	} else if (_state == State_Capture)
	{
		if (Line(robot->pos, target).distTo(ballPos) > *_escape_charge_thresh)
		{
			// Ball is in a bad place
			_state = State_Approach;
		}

		//if the ball if further away than the back off distance for the setup stage
		if(ballClose && ballPos.distTo(robot->pos) > *_drive_around_dist + Robot_Radius)
		{
			_state = State_Done;
		}

		if ( robot->hasBall() ) {
			_state = State_Done;
		}
	}
	
	// Driving
	if (_state == State_Approach)
	{
		// Move onto the line containing the ball and the_setup_ball_avoid target
		robot->addText(QString("%1").arg(targetLine.delta().dot(robot->pos - ballPos)));
		Point moveGoal = ballPos - targetLine.delta().normalized() * (*_drive_around_dist + Robot_Radius);

		static const Segment left_field_edge(Point(-Field_Width / 2.0, 0.0), Point(-Field_Width / 2.0, Field_Length));
		static const Segment right_field_edge(Point(Field_Width / 2.0, 0.0), Point(Field_Width / 2.0, Field_Length));

		// Handle edge of field case
		float field_edge_thresh = 0.3;
		Segment behind_line(ballPos - targetLine.delta().normalized() * (*_drive_around_dist),
				ballPos - targetLine.delta().normalized() * 1.0);
		state()->drawLine(behind_line);
		Point intersection;
		if (left_field_edge.nearPoint(ballPos, field_edge_thresh) && behind_line.intersects(left_field_edge, &intersection))   /// kick off left edge of far half fieldlPos, field_edge_thresh) && behind_line.intersects(left_field_edge, &intersection))
		{
			moveGoal = intersection;
		} else if (right_field_edge.nearPoint(ballPos, field_edge_thresh) && behind_line.intersects(right_field_edge, &intersection))
		{
			moveGoal = intersection;
		}

		robot->addText("Setup");
		robot->avoidBall(*_setup_ball_avoid);
		robot->move(moveGoal);

		// face in a direction so that on impact, we aim at goal
		Point delta_facing = target - ballPos;
		robot->face(robot->pos + delta_facing);

		robot->kick(0);

	} else if (_state == State_Capture)
	{
		if ( use_dribbler ) robot->dribble((*_dribble_speed));

		robot->addText("Charge!");

		state()->drawLine(robot->pos, target, Qt::white);
		state()->drawLine(ballPos, target, Qt::white);
		Point ballToTarget = (target - ballPos).normalized();
		Point robotToBall = (ballPos - robot->pos).normalized();
		Point driveDirection = robotToBall;

		// Drive directly into the ball
		double speed = min(robot->vel.mag() + (*_accel_bias * scaleAcc), _max_speed->value()); // enough of a bias to force it to accelerate
		robot->worldVelocity(driveDirection.normalized() * speed);

		// scale everything to adjust precision
		robot->setWScale(scaleW);
		robot->setVScale(scaleSpeed);

		robot->face(ballPos);

	} else {
		robot->addText("Done");
		return false;
	}

	return true;
















	// The direction we're facing
	// const Point dir = Point::direction(robot->angle * DegreesToRadians);
	
	// uint64_t now = timestamp();
	
	// // State changes
	// Point toBall = (ball().pos - robot->pos).normalized();
	// float ballDist = ball().pos.distTo(robot->pos);
	// float err = dir.dot(toBall);
	// float ballSpeed = ball().vel.mag();
	// bool behindBall = ((target - robot->pos).dot(ball().pos - robot->pos) > 0);

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
	// }

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
