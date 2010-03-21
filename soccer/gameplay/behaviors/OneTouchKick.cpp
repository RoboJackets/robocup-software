#include "OneTouchKick.hpp"

#include "../Window.hpp"

#include <cmath>

#include <boost/foreach.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace Utils;
using namespace Geometry2d;


Gameplay::Behaviors::OneTouchKick::OneTouchKick(GameplayModule *gameplay) :
	Behavior(gameplay)
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

void Gameplay::Behaviors::OneTouchKick::assign(set<Robot *> &available)
{
	_robots.clear(); // clear existing robots
	takeBest(available);

	if (!_win)
	{
		_win = new WindowEvaluator(_gameplay->state());
		_win->debug = false;
	}

	_state = Intercept;
}

bool Gameplay::Behaviors::OneTouchKick::run()
{
	// verify ball
	if (!allVisible() || !ball().valid)
	{
		// No ball
		return false;
	}

	// Get ball information
	const Geometry2d::Point ballPos = ball().pos;
	const Geometry2d::Point ballVel = ball().vel;

	// Get robot information
	const Geometry2d::Point pos = robot()->pos();
	const Geometry2d::Point vel = robot()->vel();

	// find a target segment using window evaluators
	if (targetRobot) {
		_target = evaluatePass();
	} else {
		_target = evaluateShot();
	}
	drawLine(_target, 255, 0, 0); // show the target segment

	// DEBUG: draw bezier trajectory to ball
	size_t nrPts = 20; // number of interpolation points
	float velGain = 1.0; // gain on velocity for control point
	float approachDist = 1.0; // distance along approach line for ball control point

	// define the control points
	Point c0 = pos,       // initial position
		  c1 = pos + vel * velGain, // first control point
		  c2 = ballPos + (ballPos - _target.center()).normalized()*approachDist,
		  c3 = ballPos;   // ball position
	vector<Point> controlPts;
	controlPts.push_back(c0);
	controlPts.push_back(c1);
	controlPts.push_back(c2);
	controlPts.push_back(c3);

	// create the coefficients
	vector<float> coeffs;
	for (size_t i=0; i<controlPts.size(); ++i) {
		coeffs.push_back(binomialCoefficient(controlPts.size()-1, i));
	}

	// create the points
	vector<Point> traj;
	traj.push_back(c0);
	size_t n = controlPts.size();
	float t = 0.0, inc = 1.0/nrPts;
	for (size_t i = 0; i<nrPts; ++i) {
		t += inc;
		float j = 1.0 - t;
		Point pt;
		for (size_t k = 0; k<n; ++k) {
			pt += controlPts.at(k) * pow(j, n-1-k) * pow(t, k) * coeffs.at(k);
		}
		traj.push_back(pt);
	}

	// draw the trajectory
	Point prev(-1.0, -1.0);
	BOOST_FOREACH(Point pt, traj) {
		if (prev.x != -1.0) {
			drawLine(Segment(prev, pt), 0, 0, 255);
		}
		prev = pt;
	}

	// draw the control points
	prev = Point(-1.0, -1.0);
	BOOST_FOREACH(Point pt, controlPts) {
		if (prev.x != -1.0) {
			drawLine(Segment(prev, pt), 0, 0, 255);
		}
		prev = pt;
	}

	// keep track of state transitions
	State oldState = _state;
	QColor toggle = Qt::magenta, stable = Qt::black;
	Point textOffset(Constants::Robot::Radius*1.3, 0.0);

	// HANDLE STATES (with debug text)
	switch (_state) {
	case Intercept:
		//approach the ball at high speed using Intercept
		_state = intercept();
		drawText("Intercept", robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Approach:
		_state = approach();
		drawText("Approach", robot()->pos() + textOffset, _state == oldState ? stable : toggle);
		break;
	case Done: // do nothing
		break;
	}

	return _state != Done;
}

Gameplay::Behaviors::OneTouchKick::State
Gameplay::Behaviors::OneTouchKick::intercept() {

	// if we are on the approach line, change to approach state

	return Intercept;
}

Gameplay::Behaviors::OneTouchKick::State
Gameplay::Behaviors::OneTouchKick::approach() {

	// if we have kicked the ball, we are done

	// if we have gone too far away from the approach line, go to intercept

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
	Geometry2d::Point t = targetRobot->pos();
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

int factorial(int n) {
	if ( n == 1) return 1;
	return n * factorial(n-1);
}

int Gameplay::Behaviors::OneTouchKick::binomialCoefficient(int n, int k) {
	if (k > n) throw invalid_argument("K greater than N in binomialCoefficient()!");
	if (k == n || k == 0 ) return 1;
	if (k == 1 || k == n-1) return n;

	return factorial(n)/(factorial(k)*factorial(n-k));
}
