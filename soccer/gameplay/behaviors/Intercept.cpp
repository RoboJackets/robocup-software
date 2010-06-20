#include "Intercept.hpp"

#include <iostream>
#include <Constants.hpp>

#include <cmath>

using namespace std;
using namespace Geometry2d;

Gameplay::Behaviors::Intercept::Intercept(GameplayModule *gameplay, float dist) :
	Behavior(gameplay, 1),
	_driveSide(UNSET),
	_state(ApproachFar),
	_farDist(dist),
	_ballControlFrames(5)
{
}

bool Gameplay::Behaviors::Intercept::assign(std::set<Robot *> &available) {
	takeBest(available);
	return _robots.size() >= _minRobots;
}

float Gameplay::Behaviors::Intercept::score(Robot * robot) {
	Geometry2d::Point ball_pos = ball().pos;
	Geometry2d::Point robot_pos = robot->pos();

	return ball_pos.distTo(robot_pos);
}

bool Gameplay::Behaviors::Intercept::run() {
	if (!allVisible() || !ball().valid) {
		// No ball
		return false;
	}

	if (!robot()->haveBall()) {
		_ballControlCounter = 0;
	}

	//FIXME - We ram the ball if we have to move a long distance because we don't start slowing down early enough.

	float avgVel = 0.5 * robot()->packet()->config.motion.deg45.velocity;
	float proj_thresh = 0.01;
	float proj_damp = 0.8;
	Point pos = robot()->pos(),
		  ballVel = ball().vel,
		  ballPos = ball().pos,
		  proj = (ballVel.mag() > proj_thresh) ? ballVel * (pos.distTo(ballPos)/avgVel) : Point(),
		  ballPosProj = ballPos + proj * proj_damp;

	// determine where to put the debug text
	const Geometry2d::Point textOffset(Constants::Robot::Radius * -1.3, 0);

	// Always face the ball
	robot()->face(ballPos);

	// default to full speed unless we are close to ball
	robot()->setVScale(1.0f);

	//if we already have the ball, skip approach states
	if (robot()->haveBall()) {
		if (++_ballControlCounter > _ballControlFrames) {
			_state = Done;
		} else {
			//stop forward movement
			robot()->move(robot()->pos()); // FIXME: convert to real stop
		}
	} else if (_state == ApproachBall) {
		if (!pos.nearPoint(ballPos, _farDist)) {
			_state = ApproachFar;
		}
	}

	if (_state == ApproachFar) {
		//approach the ball at high speed facing the intended direction
		robot()->willKick = false;  // want to avoid collisions

		// create extra waypoint to the side of the ball behind it
		// use hysteresis on the side of the ball
		float perp_damp = 1.0;
		Point targetTraj = (ballPos - pos).normalized();
		Point goLeft = ballPosProj + targetTraj.perpCCW().normalized() * Constants::Robot::Radius * perp_damp;
		Point goRight = ballPosProj + targetTraj.perpCW().normalized() * Constants::Robot::Radius * perp_damp;

		float leftDist = pos.distTo(goLeft);
		float rightDist = pos.distTo(goRight);

		// set the side
		float hystersis_modifier = 0.80;
		switch (_driveSide) {
		case UNSET:
			// take closest
			if (leftDist < rightDist) {
				_driveSide = LEFT;
			} else {
				_driveSide = RIGHT;
			}
			break;
		case LEFT:
			if (rightDist < hystersis_modifier * leftDist) {
				_driveSide = RIGHT;
			}
			break;
		case RIGHT:
			if (leftDist < hystersis_modifier * rightDist) {
				_driveSide = LEFT;
			}
			break;
		}

		drawText("ApproachFar", pos + textOffset, 0, 0, 0);
		Geometry2d::Point dest;
		if (_driveSide == LEFT)
			dest = goLeft;
		else if (_driveSide == RIGHT)
			dest = goRight;

		//if the ball is moving
		//we first need to try and intercept it
		if (ballVel.mag() < .1) {
			float ballDist = ballPos.distTo(pos);
			float stopDist = _farDist * (1 + robot()->vel().mag() * 0.5f);
			if (ballDist < stopDist) {
				dest += (ballPos - target).normalized() * ballDist;
			} else {
				dest += (ballPos - target).normalized() * stopDist;
			}

		}
		drawLine(Geometry2d::Segment(pos, dest), 64, 64, 255);

		robot()->move(dest);

		const float dist = dest.distTo(pos);

		if (dist <= _farDist) {
			_state = ApproachBall;
		}
	} else if (_state == ApproachBall) {
		//approach the ball with intent to acquire it
		drawText("ApproachBall", pos + textOffset, 0, 0, 0);
		//TODO change to just willHandle?
		//something more meaningful
		//we don't always want to kick
		robot()->willKick = true;

		// experimental 02/11/10
		// kickerGoalPos is the target, instead of the ballpos
		// hopefully, this will reduce ramming the ball.
		Geometry2d::Point kickerGoalPos(ballPos - target);
		kickerGoalPos = kickerGoalPos.normalized() * (Constants::Robot::Radius);
		kickerGoalPos += ballPos;

		robot()->move(kickerGoalPos);
		robot()->dribble(50);
		robot()->setVScale(0.8); // dampen velocity when near the ball

		if (robot()->haveBall()) {
			if (++_ballControlCounter > _ballControlFrames) {
				_state = Done;
			} else {
				//stop forward movement
				robot()->move(robot()->pos());
			}

		}
	} else if (_state == Done) {
		// return to full vscale
		robot()->setVScale(1.0);
		//stop forward movement
		//robot()->move(pos);
	}

	return _state != Done;
}
