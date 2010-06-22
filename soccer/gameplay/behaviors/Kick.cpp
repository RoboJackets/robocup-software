#include "Kick.hpp"

#include "../Window.hpp"

#include <boost/foreach.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace Utils;
using namespace Geometry2d;

//#define DEBUG

#ifdef DEBUG
#define debug(...) fprintf(stderr, __VA_ARGS__)
#else
#define debug(...)
#endif

Gameplay::Behaviors::Kick::Kick(GameplayModule *gameplay) :
	Behavior(gameplay, 1),
	_aimType(PIVOT),
	_kickType(KICK),
	_targetType(GOAL),
	_driveSide(UNSET),
	_ballHandlingScale(1.0),
	_ballHandlingRange(0.5)
{
	automatic = true;
	_win = 0;
	_targetRobot = 0;
	_intercept = new Gameplay::Behaviors::Intercept(gameplay);
}

Gameplay::Behaviors::Kick::~Kick()
{
	if (_win)
	{
		delete _win;
	}
	if (_intercept)
	{
		delete _intercept;
	}
}

bool Gameplay::Behaviors::Kick::kickType(KickType mode) {
	if (mode == CHIP && robot()->hasChipper()) {
		_kickType = mode;
		return true;
	} else {
		return false;
	}
}

void Gameplay::Behaviors::Kick::setTarget() {
	_targetType = GOAL;
}

void Gameplay::Behaviors::Kick::setTarget(const Geometry2d::Segment& seg) {
	_targetType = SEGMENT;
	_target = seg;
}

void Gameplay::Behaviors::Kick::setTarget(Robot * r) {
	_targetType = ROBOT;
	_targetRobot = r;
}

bool Gameplay::Behaviors::Kick::assign(set<Robot *> &available)
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

	_state = Intercept;
	_intercept->assignOne(robot());
	_lastMargin = 0;

	return _robots.size() >= _minRobots;
}

bool Gameplay::Behaviors::Kick::run()
{
	if (!allVisible() || !ball().valid)
	{
		// No ball
		return false;
	}

	// Get ball information
	const Geometry2d::Point ballPos = ball().pos;
	const Geometry2d::Point ballVel = ball().vel;

	const Geometry2d::Point pos = robot()->pos();

	// find a target segment using window evaluators
	switch (_targetType) {
	case GOAL:
		_target = evaluateShot();
		break;
	case ROBOT:
		_target = evaluatePass();
		break;
	case SEGMENT:
		_target = evaluateSegment();
	}
	drawLine(_target, 255, 0, 0); // show the target segment

	Geometry2d::Point targetCenter = _target.center();

	//Determine how hard to kick given
	int kickStrength = calcKickStrength(targetCenter);

	// Always face the ball
	robot()->face(ballPos);

	// Vector that we would shoot along if we shot now
	// assume that the ball goes where we were facing
	//Geometry2d::Point shootDir = -(pos - ballPos);
	Point shootDir = Point::direction(robot()->angle() * DegreesToRadians);

	//shot segment is from us in direction of shootDir
	//it is sufficiently large to handle the entire field shot
	Segment shotLine (pos, pos + shootDir * 7.0);

	//the point of intersection of the shotLine and target
	Geometry2d::Point shootPoint;

	//see if the shot line intersects where we indent to shoot
	bool intersectedTarget = shotLine.intersects(_target, &shootPoint);

	// robot intersection tests
	bool intersectsRobot = checkRobotIntersections(shotLine);

	// canKick is true if the robot is facing the target and the ball is between the robot and the target.
	bool canKick = intersectedTarget &&
				   robot()->pos().nearPoint(ballPos, Constants::Robot::Radius + 0.2) &&
				   !intersectsRobot &&
				   robot()->charged();

	// keep track of state transitions
	State oldState = _state;
	QColor toggle = Qt::magenta, stable = Qt::black;
	Point textOffset(Constants::Robot::Radius*1.3, 0.0);

	// NOTE: this should should only be for performing overrides
	// if intercepting and dist < aimThresh, enter aim state
	const float aimThresh = Constants::Robot::Radius + 0.05;
	// if aiming and dist > interceptThresh, enter intercept state
	const float interceptThresh = Constants::Robot::Radius + 0.4;
	// STATE TRANSITION OVERRIDES
	// check which aim mode we are in
	if (_aimType == PIVOT && (_state == OneTouchAim))
	{
		_state = Intercept;
	} else if (_aimType == ONETOUCH && (_state == Aim || _state == Shoot))
	{
		_state = OneTouchAim;
	}

	//if we already have the ball, skip approach states
//	if (_aimType == PIVOT) {
		if (_state == Intercept && robot()->haveBall())
		{
			//cout << "Intercept succeeded - switching to aim" << endl;
			_state = Aim;
			_pivot = ballPos;
		}
		else if (_state == Intercept && pos.distTo(ballPos) < aimThresh) {
			//cout << "Close enough to ball, switching to aim" << endl;
			_state = Aim;
			_pivot = ballPos;
		} else if (_state == Aim && (pos.distTo(ballPos) > interceptThresh))
		{
			_state = Intercept;
		}
//	}

	// HANDLE STATES (with debug text)
	switch (_state) {
	case Intercept:
		//approach the ball at high speed using Intercept
		_state = intercept(targetCenter);
		drawText("Intercept", robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Aim:
		_state = aim(targetCenter, canKick);
		drawText("Aim", robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Shoot:
		_state = shoot(targetCenter, kickStrength);
		drawText("Shoot", robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case OneTouchAim:
		_state = oneTouchApproach();
		drawText("OneTouchAim", robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Done: // do nothing
		break;
	}

	return _state != Done;
}

Gameplay::Behaviors::Kick::State
Gameplay::Behaviors::Kick::intercept(const Geometry2d::Point& targetCenter) {

	float avgVel = 0.5 * robot()->packet()->config.motion.deg45.velocity;
	float proj_thresh = 0.01;
	float proj_damp = 0.5;
	Point pos = robot()->pos();
	Point ballVel = ball().vel;
	Point ballPos = ball().pos;
	Point proj = (ballVel.mag() > proj_thresh) ? ballVel * (pos.distTo(ballPos)/avgVel) : Point();
	Point ballPosProj = ballPos + proj * proj_damp;

	// calculate trajectory to get to the ball
	float approachDist = 0.5; // distance along approach line for ball control point
	Point approachVec = (ballPosProj - targetCenter).normalized();

	// define the control points for a single kick
	Point approachFar  = ballPos + approachVec*approachDist;
	Point approachBall = ballPos + approachVec*Constants::Robot::Radius;
//	Point moveTarget   = Segment(ballPos, ballPos + approachVec * (0.5 * approachDist + Constants::Robot::Radius)).nearestPoint(pos);
	Point moveTarget   = ballPosProj + approachVec * (0.5 * approachDist + Constants::Robot::Radius);

	// create extra waypoint to the side of the ball behind it - use when coming in from far away
	// use hysteresis on the side of the ball
	float perp_damp = 2.0;
	Point targetTraj = (ballPosProj - pos).normalized();
	Point goLeft = ballPosProj + targetTraj.perpCCW().normalized() * Constants::Robot::Radius * perp_damp;
	Point goRight = ballPosProj + targetTraj.perpCW().normalized() * Constants::Robot::Radius * perp_damp;

	// create lines to tell if we are going the wrong way around the ball
	Segment leftLine(pos, goLeft), rightLine(pos, goRight), ballSeg(ballPos, targetCenter);

	// we always want to override the hysteresis if a line is intersecting
	// NOTE: we can't have both intersect
	if (leftLine.intersects(ballSeg)) {
		_driveSide = RIGHT;
	} else if (rightLine.intersects(ballSeg)) {
		_driveSide = LEFT;
	} else {
		// compare using path length
		float leftDist = pos.distTo(goLeft) + goLeft.distTo(moveTarget);
		float rightDist = pos.distTo(goRight) + goRight.distTo(moveTarget);

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
	}

	Point avoidDest = (_driveSide == LEFT) ? goLeft : goRight;

	// issue move command to point halfway along line
	float wrap_thresh = 0.5; // m
	if (ballPos.distTo(pos) < wrap_thresh || moveTarget.distTo(pos) < avoidDest.distTo(pos))
		robot()->move(moveTarget, false);
	else
		robot()->move(avoidDest, false);

	// face ball to ensure we hit something
	robot()->face(ballPos); // should be the targetCenter or ballPos

	// if we are in front of the ball, we should go back to intercept
	Point apprPoint = ballPos - approachVec.normalized() * Constants::Robot::Radius * 0.8;
	Segment ballPerpLine(apprPoint - approachVec.perpCW(), apprPoint + approachVec.perpCW());
	drawLine(ballPerpLine, 0, 0, 0);
	if (ballPerpLine.pointSide(ballPos) < 0.0) {
		cout << "OneTouchAim: behind robot" << endl;
		return Intercept;
	}

	// if we are on the approach line, change to approach state
	Segment approachLine(approachFar, approachBall);
	float distThresh = (_aimType == PIVOT ) ? 0.1 : 0.05;
	if (approachLine.nearPointPerp(pos, distThresh) || robot()->haveBall()) {
		robot()->willKick = true;
		return (_aimType == ONETOUCH) ? OneTouchAim : Aim;
	} else {
		robot()->willKick = false; // avoid ball when intercepting
	}

	return Intercept;
}

Gameplay::Behaviors::Kick::State
Gameplay::Behaviors::Kick::aim(const Geometry2d::Point& targetCenter, bool canKick) {
	debug("Aim: ");

	// scale velocity due to range
	if (targetCenter.distTo(robot()->pos()) <= _ballHandlingRange) {
		robot()->setVScale(_ballHandlingScale);
	}

	// prepare the robot for kicking
	robot()->willKick = true;
	robot()->dribble(30); // previously 50, set lower to prevent unintentional yanking

	const float clearance = Constants::Ball::Radius + Constants::Robot::Radius;

	// pull out current states
	float avgVel = 0.5 * robot()->packet()->config.motion.deg45.velocity;
	float proj_thresh = 0.02;
	float proj_damp = 0.3;
	Geometry2d::Point pos = robot()->pos();
	Point ballVel = ball().vel;
	Point ballPos = ball().pos;
	Point proj = (ballVel.mag() > proj_thresh) ? ballVel * (pos.distTo(ballPos)/avgVel) : Point();
	Point ballPosProj = ballPos + proj * proj_damp;

	// show an ideal line from ball to target
	drawLine(Segment(ballPosProj, targetCenter), 255, 0, 0); // red

	// show a line along the likely shot if fired now
	Segment curShotLine(pos, pos + (ballPosProj-pos).normalized()*10.0);
	Segment curShotLineViz(pos, pos + (ballPosProj-pos).normalized()*2.0);
	drawLine(curShotLineViz, 0, 0, 200); // blue

	// middle of shot arc - used for drawing text
	const Geometry2d::Point shotTextPoint = Segment(ballPosProj, targetCenter).center();

	Geometry2d::Point m = ballPosProj + (pos - ballPosProj).normalized() * clearance;
	if (pos.nearPoint(m, Constants::Robot::Radius))
	{
		debug("pivot  ");
		// We're close to the ideal kicking location, so fine tune by moving back and forth
		// to avoid getting stuck with a very small move.

		// From the ball's point of view:
		//     t is towards the target.
		//     s is perpendicular to that.
		Geometry2d::Point t = (targetCenter - ballPosProj).normalized();
		Geometry2d::Point s = t.perpCCW();

		// How far the robot is from the ball in that direction
		float d = (pos - ballPosProj).dot(s);

		// Pivot towards the target-ball line
		//robot()->pivot(_pivot, d < 0); // why not pivot around ball?
		robot()->pivot(ball().pos, d < 0); // FIXME: this is actually the wrong side occasionally
	}
	else
	{
		debug("move   ");
		robot()->move(m);
	}

	bool shotAvailable = false;
	if (canKick)
	{
		// new code using segment - just gets it as close and fires
		Point hitPoint;
		float targetRadius = 0.5 * _target.length();
		float percent_target = 0.9;
		float good_shot_thresh = percent_target * targetRadius;
		if (curShotLine.intersects(_target, &hitPoint) && hitPoint.distTo(targetCenter) < good_shot_thresh) {
			if (ballPos.nearPoint(pos, clearance + Constants::Robot::Radius))
			{
				debug("Shoot\n");
				_shootStart = pos;
				_shootMove = pos + (ballPosProj - pos).normalized() * 0.2f;
				_shootBallStart = ballPosProj;
				shotAvailable = true;
			}
		}
	}

	// determine return state
	return shotAvailable ? Shoot : Aim;
}

Gameplay::Behaviors::Kick::State
Gameplay::Behaviors::Kick::shoot(const Geometry2d::Point& targetCenter, int kickStrength) {
	debug("Shoot: %f", ball().vel.dot(robot()->pos() - ball().pos));

	if (_kickType == KICK)
		robot()->kick(kickStrength);
	else if (_kickType == CHIP)
		robot()->chip(kickStrength);

	robot()->face(targetCenter);
	robot()->dribble(0);
	robot()->move(_shootMove);

	if (!robot()->pos().nearPoint(ball().pos, 0.2f))
	{
		// Lost the ball
		debug(" Lost ball");
		return Aim;
	}

	if (!robot()->charged())
	{
		// We have kicked
		return Done;
	}

	// If the robot has moved more than half a meter since starting to shoot, we are probably
	// just pushing the ball around, so give up.
	if (!ball().pos.nearPoint(_shootBallStart, 0.2f) || !robot()->pos().nearPoint(_shootStart, 0.5f))
	{
		// Ball was kicked or we moved far enough to give up
//		return Done;
	}
	debug("\n");
	return Shoot;
}

Gameplay::Behaviors::Kick::State
Gameplay::Behaviors::Kick::oneTouchApproach() {

	// if we have kicked the ball, we are done
	if (robot()->charged() == false) {
		robot()->willKick = false;
		return Done;
	}

	float avgVel = 0.5 * robot()->packet()->config.motion.deg45.velocity;
	float proj_thresh = 0.01;
	Point pos = robot()->pos(),
		  vel = robot()->vel(),
		  ballVel = ball().vel,
		  ballPos = ball().pos,
		  proj = (ballVel.mag() > proj_thresh) ? ballVel * (pos.distTo(ballPos)/avgVel) : Point(),
		  ballPosProj = ballPos + proj,
		  approachVec = (ballPos - _target.center()).normalized();

	// if the ball is suddenly moving now, we have hit/kicked it, so back off
	float kickThresh = 1.0;
	if (ball().accel.mag() > kickThresh) {
		return Done;
	}

	// if we are in front of the ball, we should go back to intercept
	Point apprPoint = ballPos - approachVec.normalized() * Constants::Robot::Radius * 0.8;
	Segment ballPerpLine(apprPoint - approachVec.perpCW(), apprPoint + approachVec.perpCW());
	drawLine(ballPerpLine, 0, 0, 0);
	if (ballPerpLine.pointSide(ballPos) < 0.0) {
		cout << "OneTouchAim: behind robot" << endl;
		return Intercept;
	}

	// turn on the kicker for final approach
	float fire_kick_thresh = Constants::Robot::Radius + Constants::Ball::Radius + 0.10;
	if (pos.distTo(ballPos) < fire_kick_thresh)
		robot()->kick(calcKickStrength(_target.center()));

	// calculate trajectory to hit the ball correctly
	float approachDist = 2.0; // how long to extend approach line beyond ball

	// define the control points for a single kick
	Point approachFar = ballPos + approachVec * Constants::Robot::Radius + proj,
		  approachBall = ballPos - approachVec * approachDist * Constants::Robot::Radius + proj;

	// use a 3rd degree bezier curve to get to the ball
	_controls.clear();
	_controls.push_back(pos);          // start at robot position
	_controls.push_back(approachFar);  // back point for approach line
	_controls.push_back(approachBall); // extended point

	// issue move command if we don't need to change states
	robot()->bezierMove(_controls, Packet::MotionCmd::Endpoint);

	// if we have gotten too far away (given hysteresis), go back to intercept
	Segment approachLine(approachFar, approachBall);
	float distThresh = 0.25;
	if (!approachLine.nearPoint(pos, distThresh)) {
//		cout << "OneTouchApproach: " << approachLine.distTo(pos) << " too far away from approach line, switching to intercept" << endl;
		return Intercept;
	}

	return OneTouchAim; // continue aiming
}

float Gameplay::Behaviors::Kick::score(Robot* robot)
{
	return (robot->pos() - ball().pos).magsq();
}

Geometry2d::Segment Gameplay::Behaviors::Kick::evaluatePass() {
	// Update the target window
	_win->exclude.clear();
	_win->exclude.push_back(robot()->pos());
	_win->debug = true;

	// Kick towards a robot
	Geometry2d::Point t = _targetRobot->pos();
	_win->run(ball().pos, t);
	_win->exclude.push_back(t);

	Segment target;
	if (_win->best)
	{
		target = _win->best->segment;
	} else {
		// Can't reach the target
		target = _win->target();
	}
	return target;
}

Geometry2d::Segment Gameplay::Behaviors::Kick::evaluateShot() {
	// Update the target window
	_win->exclude.clear();
	_win->exclude.push_back(robot()->pos());
	_win->debug = true;

	//goal line, for intersection detection
	Segment target(Point(-Constants::Field::GoalWidth / 2.0f, Constants::Field::Length),
			Point(Constants::Field::GoalWidth / 2.0f, Constants::Field::Length));

	// Try to kick to the goal.
	_win->run(ball().pos, target);

	if (!_win->best && automatic)
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

Geometry2d::Segment Gameplay::Behaviors::Kick::evaluateSegment() {
	// Update the target window
	_win->exclude.clear();
	_win->exclude.push_back(robot()->pos());
	_win->debug = true;

	//goal line, for intersection detection
	Segment target = _target;

	// Try to kick to the goal.
	_win->run(ball().pos, target);

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

int Gameplay::Behaviors::Kick::calcKickStrength(const Geometry2d::Point& targetCenter) {
	//if kicking to a target
	//calculate kick strength
	int kickStrength = 255;
	if (_targetType == ROBOT)
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

//FIXME - These robot equivalence checks are too complicated.
// Should be able to compare pointers.  Also don't use LogFrame from within Gameplay.
bool Gameplay::Behaviors::Kick::checkRobotIntersections(const Geometry2d::Segment& shotLine) {
	bool intersectsRobot = false;
	BOOST_FOREACH(const Packet::LogFrame::Robot& r, gameplay()->state()->opp)
	{
		//don't check against if target
		bool noAdd = (_targetType == ROBOT && _targetRobot && !_targetRobot->self() && _targetRobot->packet()->shell == r.shell);
		if (r.valid && !noAdd)
		{
			if (shotLine.intersects(Circle(r.pos, Constants::Robot::Radius + Constants::Ball::Radius)))
			{
				intersectsRobot = true;
				break;
			}
		}
	}

	BOOST_FOREACH(const Packet::LogFrame::Robot& r, gameplay()->state()->self)
	{
		//don't check against if target
		bool noAdd = (_targetType == ROBOT && _targetRobot && _targetRobot->self() && _targetRobot->packet()->shell == r.shell);
		if (r.valid && r.shell != robot()->packet()->shell && !noAdd)
		{
			if (shotLine.intersects(Circle(r.pos, Constants::Robot::Radius + Constants::Ball::Radius)))
			{
				intersectsRobot = true;
				break;
			}
		}
	}
	return intersectsRobot;
}

