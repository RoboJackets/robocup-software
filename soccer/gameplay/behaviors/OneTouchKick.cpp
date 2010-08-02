#include "OneTouchKick.hpp"

#include "../Window.hpp"

using namespace std;
using namespace Utils;
using namespace Geometry2d;

Gameplay::Behaviors::OneTouchKick::OneTouchKick(GameplayModule *gameplay) :
	Behavior(gameplay), _kickType(KICK)
{
	_win = 0;
	targetRobot = 0;
}

Gameplay::Behaviors::OneTouchKick::~OneTouchKick()
{
	if (_win)
	{
		delete _win;
	}
}

bool Gameplay::Behaviors::OneTouchKick::assign(set<Robot *> &available)
{
	_robots.clear(); // clear existing robots
	if (!takeBest(available))
	{
	    return false;
	}

	if (!_win)
	{
		_win = new WindowEvaluator(_gameplay->state());
		_win->debug = false;
	}

	cout << "Initializing OneTouchKick to Intercept" << endl;
	_state = Intercept;
	_commandValid = false;

	return true;
}

bool Gameplay::Behaviors::OneTouchKick::kickType(KickType mode) {
	if (mode == CHIP && robot()->hasChipper()) {
		_kickType = mode;
		return true;
	} else {
		return false;
	}
}

bool Gameplay::Behaviors::OneTouchKick::run()
{
	// verify ball
	if (!allVisible() || !ball().valid)
	{
		// No ball
		return false;
	}

	// find a target segment using window evaluators
	if (targetRobot) {
		_target = evaluatePass();
	} else {
		_target = evaluateShot();
	}
	state()->drawLine(_target, Qt::red); // show the target segment

	// keep track of state transitions
	State oldState = _state;
	QColor toggle = Qt::magenta, stable = Qt::black;
	Point textOffset(Constants::Robot::Radius*1.3, 0.0);

	// HANDLE STATES (with debug text)
	switch (_state) {
	case Intercept:
		//approach the ball at high speed using Intercept
		_state = intercept();
		state()->drawText("Intercept", robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Approach:
		_state = approach();
		state()->drawText("Approach", robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Done: // do nothing
		break;
	}

	return _state != Done;
}

Gameplay::Behaviors::OneTouchKick::State
Gameplay::Behaviors::OneTouchKick::intercept() {
	// calculate trajectory to get to the ball
	float approachDist = 0.5; // distance along approach line for ball control point

	Point pos = robot()->pos(), ballPos = ball().pos;
	Point approachVec = (ballPos - _target.center()).normalized();

	// define the control points for a single kick
	Point approachFar  = ballPos + approachVec*approachDist,
		  approachBall = ballPos + approachVec*Constants::Robot::Radius;   // ball position

	// use a 3rd degree bezier curve to get to the ball
	_controls.clear();
	_controls.push_back(pos);          // start at robot position
	_controls.push_back(approachFar);  // back point for approach line
	_controls.push_back(approachBall); // target destination

	// issue move command
	robot()->bezierMove(_controls, MotionCmd::Continuous);

	// if we are not able to kick stay in the intercept
	if (!robot()->charged())
		return Intercept;

	// if we are on the approach line, change to approach state
	Segment approachLine(approachFar, approachBall);
	float distThresh = 0.1;
	if (approachLine.nearPoint(pos, distThresh)) {
		return Approach;
	}

	// by default, continue in state
	return Intercept;
}

Gameplay::Behaviors::OneTouchKick::State
Gameplay::Behaviors::OneTouchKick::approach() {

	// if we have kicked the ball, we are done
	if (robot()->charged() == false) {
		robot()->willKick = false;
		return Done;
	}

	Point pos = robot()->pos(), ballPos = ball().pos;
	Point approachVec = (ballPos - _target.center()).normalized();

	// if the ball is suddenly moving now, we have hit/kicked it, so back off
	float kickThresh = 1.0;
	if (ball().accel.mag() > kickThresh) {
		return Done;
	}

	// if we are in front of the ball, we should go back to intercept
	Point apprPoint = ballPos + approachVec * Constants::Robot::Radius * 0.8;
	Segment ballPerpLine(apprPoint - approachVec.perpCW(), apprPoint + approachVec.perpCW());
	state()->drawLine(ballPerpLine);
	if (ballPerpLine.pointSide(ballPos) > 0.0)
	{
		return Intercept;
	}

	// turn on the kicker for final approach
	robot()->willKick = true;
	if (_kickType == KICK)
		robot()->kick(calcKickStrength(_target.center()));
	else if (_kickType == CHIP)
		robot()->chip(calcKickStrength(_target.center()));

	// calculate trajectory to hit the ball correctly
	float approachDist = 2.0; // how long to extend approach line beyond ball

	// define the control points for a single kick
	Point approachFar = ballPos + approachVec * Constants::Robot::Radius,
		  approachBall = ballPos - approachVec * approachDist * Constants::Robot::Radius;

	// use a 3rd degree bezier curve to get to the ball
	_controls.clear();
	_controls.push_back(pos);          // start at robot position
	_controls.push_back(approachFar);  // back point for approach line
	_controls.push_back(approachBall); // extended point

	// issue move command if we don't need to change states
	robot()->bezierMove(_controls, MotionCmd::Endpoint);

	// if we have gotten too far away (given hysteresis), go back to intercept
	Segment approachLine(approachFar, approachBall);
	float distThresh = 0.15;
	if (!approachLine.nearPoint(pos, distThresh)) {
		return Intercept;
	}

	return Approach;
}

float Gameplay::Behaviors::OneTouchKick::score(Robot* robot)
{
	return (robot->pos() - ball().pos).magsq();
}

Geometry2d::Segment Gameplay::Behaviors::OneTouchKick::evaluatePass() {
	// Update the target window
	_win->exclude.clear();
	_win->exclude.push_back(robot()->pos());
	_win->debug = true;

	// Kick towards a robot
	Geometry2d::Segment target = targetRobot->kickerBar();
	_win->run(ball().pos, target);
	_win->exclude.push_back(targetRobot->pos());

	if (_win->best)
	{
		target = _win->best->segment;
	} else {
		// Can't reach the target
		target = _win->target();
	}
	return target;
}

Geometry2d::Segment Gameplay::Behaviors::OneTouchKick::evaluateShot() {
	// Update the target window
	_win->exclude.clear();
	_win->exclude.push_back(robot()->pos());
	_win->debug = true;

	//goal line, for intersection detection
	Segment target(Point(-Constants::Field::GoalWidth / 2.0f, Constants::Field::Length),
			Point(Constants::Field::GoalWidth / 2.0f, Constants::Field::Length));

	// Try to kick to the goal.
	_win->run(ball().pos, target);

	if (!_win->best )
	{
		// Use the entire end line
		target.pt[0] = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length);
		target.pt[1] = Geometry2d::Point(Constants::Field::Width / 2.0, Constants::Field::Length);

		_win->run(ball().pos, target);
		if (!_win->best)
		{
			// Use the last 1/3rd of a side
			if (ball().pos.x > 0)
			{
				target.pt[0] = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length);
				target.pt[1] = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length * 2 / 3);
			} else {
				target.pt[0] = Geometry2d::Point(Constants::Field::Width / 2.0, Constants::Field::Length);
				target.pt[1] = Geometry2d::Point(Constants::Field::Width / 2.0, Constants::Field::Length * 2 / 3);
			}
			_win->run(ball().pos, target);
		}
	}

	Segment bestTarget;
	if (_win->best)
	{
		bestTarget = _win->best->segment;
	} else {
		// Can't reach the target
		bestTarget = _win->target();
	}
	return bestTarget;
}

int Gameplay::Behaviors::OneTouchKick::calcKickStrength(const Geometry2d::Point& targetCenter) const {
	//if kicking to a target
	//calculate kick strength
	int kickStrength = 255;
	if (targetRobot)
	{
		const float dist = robot()->pos().distTo(targetCenter);

		const float m = robot()->packet()->config.kicker.m;
		const float b = robot()->packet()->config.kicker.b;

		kickStrength = int(m * dist + b);

		if (kickStrength < 0)
		{
			kickStrength = 0;
		}
		else if (kickStrength > 255)
		{
			kickStrength = 255;
		}
	}
	return kickStrength;
}
