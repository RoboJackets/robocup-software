#include "Yank.hpp"

#include <Utils.hpp>

using namespace std;
using namespace Geometry2d;

namespace Gameplay
{
	namespace Behaviors
	{
		REGISTER_CONFIGURABLE(Yank)
	}
}

ConfigDouble *Gameplay::Behaviors::Yank::_yank_travel_thresh;
ConfigDouble *Gameplay::Behaviors::Yank::_max_aim_error;
ConfigDouble *Gameplay::Behaviors::Yank::_backup_dist;
ConfigDouble *Gameplay::Behaviors::Yank::_ball_clearance;
ConfigDouble *Gameplay::Behaviors::Yank::_bump_distance;

void Gameplay::Behaviors::Yank::createConfiguration(Configuration* cfg)
{
	_yank_travel_thresh  = new ConfigDouble(cfg, "Yank/Ball Travel Threshold", 0.5);
	_max_aim_error  = new ConfigDouble(cfg, "Yank/Ball Max Trajectory Error", 0.3);
	_backup_dist  = new ConfigDouble(cfg, "Yank/Backup Distance", 0.5);
	_ball_clearance  = new ConfigDouble(cfg, "Yank/Ball Clearance", Robot_Radius + Ball_Radius + 0.1);
	_bump_distance  = new ConfigDouble(cfg, "Yank/Bump Travel Distance", 0.3);
}

Gameplay::Behaviors::Yank::Yank(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay), _capture(gameplay)
{
	restart();
	
	target.pt[0] = Point(Field_GoalWidth / 2, Field_Length);
	target.pt[1] = Point(-Field_GoalWidth / 2, Field_Length);
}

void Gameplay::Behaviors::Yank::restart()
{
	_state = State_Capture;
	_capture.restart();
	_capture.target = target.center();
	enable_yank = true;
	enable_bump = false;
	dribble_speed = 127;  // use maximum dribbler speed by default
}

bool Gameplay::Behaviors::Yank::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}
	
	// force sub-behavior to have its robot assigned
	_capture.robot = robot;

	// The direction we're facing
	// const Point dir = Point::direction(robot->angle * DegreesToRadians);
	
	// State changes
	Point targetToBall = (ball().pos - target.center()).normalized();
	Segment yankLine(target.center(), ball().pos), aimLine(ball().pos, ball().pos + targetToBall * 5.0);
	if (_state == State_Capture)
	{
		if (_capture.done() && enable_yank)
		{
			_yankBallStart = ball().pos;
			_yankRobotStart = robot->pos;
			if (enable_bump)
			{
				_state = State_Bump;
			} else
			{
				_state = State_Yank;
			}
		}
	} else if (_state == State_Bump)
	{
		if (!_yankRobotStart.nearPoint(robot->pos, *_bump_distance))
		{
			_state = State_Yank;
		}
	} else if (_state == State_Yank)
	{
		// we are done if ball has moved more than thresh past start point towards target
		Segment fixedYankLine(target.center(), _yankBallStart);
		if (!ball().pos.nearPoint(_yankBallStart, *_yank_travel_thresh) && fixedYankLine.nearPoint(ball().pos, *_max_aim_error))
		{
			_state = State_Done;
		}

		// if ball has gotten away, reset to capture
		if (!fixedYankLine.nearPoint(ball().pos, *_max_aim_error)) {
			_state = State_Capture;
		}
	}
	
	Segment fixedYankLine(target.center(), _yankBallStart);
	state()->drawLine(yankLine, Qt::red);
	state()->drawLine(aimLine, Qt::black);
	state()->drawLine(target, Qt::yellow);
	
	// Driving
	if (_state == State_Capture)
	{
		robot->addText("Capturing");

		// aim in opposite direction
		_capture.target = aimLine.pt[1]; // opposite direction
		_capture.run();
	}  else if (_state == State_Bump)
	{
		robot->addText("Bumping");
		robot->avoidBallRadius(-1.0);
		robot->worldVelocity(fixedYankLine.delta() * -5.0);
		robot->angularVelocity(0.0);

	}  else if (_state == State_Yank)
	{
		robot->addText("Yanking");
		state()->drawLine(fixedYankLine, Qt::white);
		
		// if we are close to the ball, we must back up fast
		if (ball().pos.nearPoint(robot->pos, *_backup_dist)) {
			robot->avoidBallRadius(-1.0);
			robot->worldVelocity(fixedYankLine.delta() * -5.0);
			robot->angularVelocity(0.0);
		} else
		{
			// otherwise, get off of the yank line with path planning
			robot->avoidBallRadius(0.3);
			// don't drive into the ball
			robot->localObstacles(std::shared_ptr<Obstacle>(new CircleObstacle(_yankBallStart, ball().pos.distTo(_yankBallStart) + Robot_Radius)));
			// stay out of path of ball
			robot->localObstacles(std::shared_ptr<Obstacle>(new PolygonObstacle(Polygon(fixedYankLine, *_ball_clearance))));
			robot->move(robot->pos, false); // let path planning clear the line
		}

		robot->dribble(dribble_speed);
	} else {
		robot->addText("Done");
		return false;
	}
	
	return true;
}
