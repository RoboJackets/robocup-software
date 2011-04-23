#include <Robot.hpp>
#include <gameplay/planning/bezier.hpp>
#include <framework/Dynamics.hpp>
#include <Utils.hpp>
#include <LogUtils.hpp>
#include <protobuf/LogFrame.pb.h>

#include <stdio.h>
#include <iostream>
#include <execinfo.h>
#include <stdexcept>
#include <boost/foreach.hpp>

using namespace std;
using namespace Geometry2d;

/** Constant for timestamp to seconds */
const float intTimeStampToFloat = 1000000.0f;

//The threshold necessary to change paths tuning required 
const float path_threshold = 2 * Robot_Diameter;

const bool verbose = false;

Robot::Robot(unsigned int shell, bool self)
{
	_shell = shell;
	_self = self;
	angle = 0;
	angleVel = 0;
}

OurRobot::OurRobot(int shell, SystemState *state):
					Robot(shell, true),
					_state(state)
{
	_ball_avoid = AVOID_SMALL;
	_ball_avoid_radius = Ball_Radius;
	_delayed_goal = boost::none;
	exclude = false;
	hasBall = false;
	sensorConfidence = 0;
	cmd_w = 0;
	_lastChargedTime = 0;

	_dynamics = new Planning::Dynamics(this);
	_planner = new Planning::RRT::Planner();
	for (size_t i = 0; i < Num_Shells; ++i)
	{
		approachOpponent[i] = false;
		// TODO move thresholds elsewhere
		_self_avoid_mask[i] = (i != (size_t) shell) ? Robot_Radius : -1.0;
		_opp_avoid_mask[i] = Robot_Radius - 0.03;
	}

	_planner->setDynamics(_dynamics);
	_planner->maxIterations(250);

	radioTx.set_board_id(shell);
	for (int m = 0; m < 4; ++m)
	{
		radioTx.add_motors(0);
	}
}

void OurRobot::addText(const QString& text, const QColor& qc)
{
	Packet::DebugText *dbg = new Packet::DebugText;
	QString layer = QString("RobotText%1").arg(shell());
	dbg->set_layer(_state->findDebugLayer(layer));
	dbg->set_text(text.toStdString());
	dbg->set_color(color(qc));
	robotText.push_back(dbg);
}

void OurRobot::setCommandTrace()
{
	void *trace[9];
	int n = backtrace(trace, sizeof(trace) / sizeof(trace[0]));

	// Skip the call into this function
	_commandTrace.resize(n - 1);
	for (int i = 0; i < n - 1; ++i)
	{
		_commandTrace[i] = trace[i + 1];
	}
}

bool OurRobot::avoidOpponents() const {
	// checks for avoiding all opponents
	for (size_t i=0; i<Num_Shells; ++i)
		if (_state->opp[i] && _state->opp[i]->visible && _opp_avoid_mask[i] < 0.1)
			return false;
	return true;
}

void OurRobot::avoidOpponents(bool enable) {
	BOOST_FOREACH(float a, _opp_avoid_mask)
		if (enable) a = Robot_Radius - 0.03;
		else a = -1.0;
}

bool OurRobot::willKick() const {
	return _ball_avoid == OurRobot::KICK;
}

void OurRobot::willKick(bool enable) {
	if (enable)
		_ball_avoid = OurRobot::KICK;
	else
		_ball_avoid = OurRobot::AVOID_SMALL;
}

bool OurRobot::avoidBallLarge() const {
	return _ball_avoid == OurRobot::AVOID_LARGE;
}

void OurRobot::avoidBallLarge(bool enable) {
	if (enable)
		_ball_avoid = OurRobot::AVOID_LARGE;
	else
		_ball_avoid = OurRobot::AVOID_SMALL;
}

bool OurRobot::avoidBall() const {
	return _ball_avoid == AVOID_PARAM;
}

void OurRobot::avoidBall(bool enable, float radius) {
	if (!enable)
		_ball_avoid = AVOID_NONE;
	else {
		_ball_avoid = AVOID_PARAM;
		_ball_avoid_radius = radius;
	}
}

float OurRobot::avoidBallRadius() const {
	return _ball_avoid_radius;
}

void OurRobot::ballAvoidance(const BallAvoid& flag, boost::optional<float> radius) {
	_ball_avoid = flag;
	if (radius)
		_ball_avoid_radius = *radius;
}

void OurRobot::resetMotionCommand()
{
	if (verbose && visible) cout << "in OurRobot::resetMotionCommand()" << endl;
	robotText.clear();

	// FIXME: these are moved to assignment to allow for commands from the previous frame to
	// still be in effect.  They are automatically reset at assignment by assignNearest()
	//	willKick = false;
	//	avoidBall = false;

	radioTx.set_roller(0);
	radioTx.set_kick(0);
	radioTx.set_use_chipper(false);

	for (int i = 0; i < 4; ++i)
	{
		radioTx.set_motors(i, 0);
	}

	cmd = MotionCmd();

	_local_obstacles.clear();

}

void OurRobot::stop()
{
	_delayed_goal = boost::none;
}

void OurRobot::avoid(const BallAvoid& ball, const RobotMask& opp_robots,
		const RobotMask& self_robots, const ObstacleGroup& regions) {
	_ball_avoid = ball;
	_local_obstacles = regions;
	_opp_avoid_mask = opp_robots;

	// override the mask for this robot to ensure
	_self_avoid_mask = self_robots;
	_self_avoid_mask[shell()] = -1.0;
}

void OurRobot::move(Geometry2d::Point goal, bool stopAtEnd)
{
	if (!visible)
		return;

	// sets flags for future movement
	if (verbose) cout << " in OurRobot::move(goal): adding a goal (" << goal.x << ", " << goal.y << ")" << endl;
	addText(QString("move:(%1, %2)").arg(goal.x).arg(goal.y));
	_delayed_goal = goal;
	_planner_type = RRT;
	cmd.pathEnd = (stopAtEnd) ? MotionCmd::StopAtEnd : MotionCmd::FastAtEnd;
}

void OurRobot::move(const vector<Geometry2d::Point>& path, bool stopAtEnd)
{
	_state->drawLine(path.back(), pos);

	// copy path from input
	_path.clear();
	_path.points = path;

	// ensure RRT not used
	_delayed_goal = boost::none;
	_planner_type = OVERRIDE;

	// convert to motion command
	cmd.goalPosition = findGoalOnPath(pos, _path);
	cmd.pathLength = _path.length(pos);
	cmd.planner = MotionCmd::Point;
	cmd.pathEnd = (stopAtEnd) ? MotionCmd::StopAtEnd : MotionCmd::FastAtEnd;
}

void OurRobot::bezierMove(const vector<Geometry2d::Point>& controls,
		MotionCmd::OrientationType facing,
		MotionCmd::PathEndType endpoint) {

	// calculate path using simple interpolation
	//	_path = Planning::createBezierPath(controls);

	// execute path
	//executeMove(endpoint); // FIXME: handles curves poorly


	size_t degree = controls.size();

	// generate coefficients
	vector<float> coeffs;
	for (size_t i=0; i<degree; ++i) {
		coeffs.push_back(Planning::binomialCoefficient(degree-1, i));
	}

	// calculate length to allow for determination of time
	double pathLength = Planning::bezierLength(controls, coeffs);

	// calculate numerical derivative by stepping ahead a fixed constant
	float lookAheadDist = 0.15; // in meters along path
	float dt = lookAheadDist/pathLength;

	float velGain = 3.0; // FIXME: should be dependent on the length of the curve

	// calculate a target velocity for translation
	Point targetVel = Planning::evaluateBezierVelocity(dt, controls, coeffs);

	// apply gain
	targetVel *= velGain;

	// create a dummy goal position
	cmd.goalPosition = pos + targetVel;
	cmd.pathLength = pathLength;
	cmd.planner = MotionCmd::Point;
}

void OurRobot::directVelocityCommands(const Geometry2d::Point& trans, double ang)
{
	// ensure RRT not used
	_delayed_goal = boost::none;
	_planner_type = OVERRIDE;

	cmd.planner = MotionCmd::DirectVelocity;
	cmd.direct_ang_vel = ang;
	cmd.direct_trans_vel = trans;
}

void OurRobot::directMotorCommands(const vector<int8_t>& speeds) {
	// ensure RRT not used
	_delayed_goal = boost::none;
	_planner_type = OVERRIDE;

	cmd.planner = MotionCmd::DirectMotor;
	cmd.direct_motor_cmds = speeds;
}

Geometry2d::Point OurRobot::pointInRobotSpace(const Geometry2d::Point& pt) const {
	Point p = pt;
	p.rotate(pos, -angle);
	return p;
}

const Geometry2d::Segment OurRobot::kickerBar() const {
	TransformMatrix pose(pos, angle);
	const float mouthHalf = Robot_MouthWidth/2.0f;
	float x = sin(acos(mouthHalf/Robot_Radius))*Robot_Radius;
	Point L(x, Robot_MouthWidth/2.0f);
	Point R(x, -Robot_MouthWidth/2.0f);
	return Segment(pose*L, pose*R);
}

bool OurRobot::behindBall(const Geometry2d::Point& ballPos) const {
	Point ballTransformed = pointInRobotSpace(ballPos);
	return ballTransformed.x < -Robot_Radius;
}


void OurRobot::setVScale(float scale) {
	cmd.vScale = scale;
}

void OurRobot::setWScale(float scale) {
	cmd.wScale = scale;
}

float OurRobot::kickTimer() const {
	return (charged()) ? 0.0 : intTimeStampToFloat * (float) (Utils::timestamp() - _lastChargedTime);
}

void OurRobot::update() {
	if (charged())
	{
		_lastChargedTime = Utils::timestamp();
	}
}

void OurRobot::spin(MotionCmd::SpinType dir)
{
	cmd.spin = dir;
}

bool OurRobot::hasChipper() const
{
	return false;
}

void OurRobot::dribble(int8_t speed)
{
	radioTx.set_roller(speed);
}

void OurRobot::pivot(Geometry2d::Point ctr, MotionCmd::PivotType dir)
{
	cmd.pivotPoint = ctr;
	cmd.pivot = dir;
}

void OurRobot::face(Geometry2d::Point pt, bool continuous)
{
	cmd.goalOrientation = pt;
	cmd.face = continuous ? MotionCmd::Endpoint : MotionCmd::Continuous;
}

void OurRobot::faceNone()
{
	cmd.face = MotionCmd::None;
}

void OurRobot::kick(uint8_t strength)
{
	_ball_avoid = KICK;
	radioTx.set_kick(strength);
	radioTx.set_use_chipper(false);
}

void OurRobot::chip(uint8_t strength)
{
	_ball_avoid = KICK;
	radioTx.set_kick(strength);
	radioTx.set_use_chipper(true);
}

void OurRobot::pivot(Geometry2d::Point center, bool cw)
{
	cmd.pivotPoint = center;
	cmd.pivot = cw ? MotionCmd::CW : MotionCmd::CCW;
}

bool OurRobot::charged() const
{
	return radioRx.charged();
}

void OurRobot::approachOpp(Robot * opp, bool value) {
	approachOpponent[opp->shell()] = value;
}

ObstaclePtr OurRobot::createBallObstacle() const {
	// no obstacle when kicking is enabled
	if (_ball_avoid == KICK)
		return ObstaclePtr();

	// choose size of the obstacle
	float radius = Ball_Radius;
	switch (_ball_avoid) {
		case OurRobot::AVOID_LARGE:
			radius = Field_CenterRadius;
			break;
		case OurRobot::AVOID_PARAM:
			radius = _ball_avoid_radius;
			break;
		default:
			break;
	}
	return ObstaclePtr(new CircleObstacle(_state->ball.pos, radius));
}

Geometry2d::Point OurRobot::findGoalOnPath(const Geometry2d::Point& pose,
		const Planning::Path& path,	const ObstacleGroup& obstacles) {
		setCommandTrace();

		// TODO: verify the use of going from the closest point - previously started at beginning

		// empty path case - leave robot stationary
		if (path.empty())
			return pose;

		// path properties
		float length = path.length(0);

		// handle direct point commands where the length may be very small
		if (length < 1e-5)
			length = pos.distTo(path.points[0]);

		// go to nearest point if only point or closest point is the goal
		if (path.size() == 1)
			return path.points[0];

		// can't mix, just go to endpoint
		if (path.size() == 2)
			return path.points[1];

		// All other cases: proportionally blend the next two points together for a smoother
		// path, so long as it is still viable

		// pull out relevant points
		Point p0 = pos,
				  p1 = path.points[1],
				  p2 = path.points[2];
		float dist1 = p1.distTo(p1), dist2 = p1.distTo(p2);

		// mix the next point between the first and second point
		float scale = 1-Utils::clamp(dist1/dist2, 1.0, 0.0);
		Geometry2d::Point targetPos = p1 + (p2-p1)*scale;

		// check for collisions on blended path
		Planning::Path smoothPath;
		smoothPath.points.push_back(p0);
		smoothPath.points.push_back(targetPos);
		if (smoothPath.hit(obstacles, 0))
			return p1;
		else
			return targetPos;


		// version that tries to start at closest
//		// empty path case - leave robot stationary
//		if (path.empty())
//			return pose;
//
//		// path properties
//		float length = path.length(pose);
//		const size_t start_idx = path.nearestIndex(pose);
//		Geometry2d::Point start_pt = path.points[start_idx];
//		// if we are starting at the beginning of the path, start_idx is a useless goal
//		bool close = start_pt.nearPoint(pose, 0.1);
//
//		// handle direct point commands where the length may be very small
//		if (length < 1e-5)
//			length = pos.distTo(start_pt);
//
//		// go to nearest point if only point or closest point is the goal
//		if (path.size() == 1 || start_idx == path.size()-1)
//			return start_pt;
//
//		// can't mix, just go to endpoint
//		if (close && (path.size() == 2 || start_idx == path.size()-2))
//			return path.points[start_idx+1];
//
//		// All other cases: proportionally blend the next two points together for a smoother
//		// path, so long as it is still viable
//
//		// pull out relevant points
//		Point p0 = pos,
//				  p1 = (close) ? path.points[start_idx+1] : path.points[start_idx],
//				  p2 = (close) ? path.points[start_idx+2] : path.points[start_idx+1];
//		float dist1 = p1.distTo(p1), dist2 = p1.distTo(p2);
//
//		// mix the next point between the first and second point
//		float scale = 1-Utils::clamp(dist1/dist2, 1.0, 0.0);
//		Geometry2d::Point targetPos = p1 + (p2-p1)*scale;
//
//		// check for collisions on blended path
//		Planning::Path smoothPath;
//		smoothPath.points.push_back(p0);
//		smoothPath.points.push_back(targetPos);
//		if (smoothPath.hit(obstacles, 0))
//			return p1;
//		else
//			return targetPos;
}

Planning::Path OurRobot::rrtReplan(const Geometry2d::Point& goal,
		const ObstacleGroup& obstacles) {
	setCommandTrace();

	// create a new path
	Planning::Path result;

	// run the RRT planner to generate a new plan
	_planner->run(pos, angle, vel, goal, &obstacles, result);

	return result;
}

Geometry2d::Point OurRobot::escapeObstacles(const Geometry2d::Point& pose,
		const ObstacleGroup& hitset) const {
	// PLACEHOLDER: only escapes from one obstacle at a time
	ObstaclePtr obs = *hitset.begin();
	return obs->closestEscape(pose);
}

void OurRobot::drawPath(const Planning::Path& path, const QColor &color) {
	Geometry2d::Point last = pos;
	BOOST_FOREACH(Geometry2d::Point pt, path.points)
	{
		_state->drawLine(last, pt, color);
		last = pt;
	}
}

void OurRobot::execute(const ObstacleGroup& global_obstacles) {
	setCommandTrace();

	// if motion command complete and now allowment for planning - we're done
	if (_planner_type != OurRobot::RRT) {
		if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: non-RRT planner" << endl;
		return;
	}

	// create and visualize obstacles
	ObstacleGroup full_obstacles(_local_obstacles);
	ObstacleGroup
		self_obs = createRobotObstacles(_state->self, _self_avoid_mask),
		opp_obs = createRobotObstacles(_state->opp, _opp_avoid_mask);
	ObstaclePtr ball_obs = createBallObstacle();
	_state->drawObstacles(self_obs, Qt::gray, "self_obstacles");
	_state->drawObstacles(opp_obs, Qt::gray, "opp_obstacles");
	_state->drawObstacle(ball_obs, Qt::gray, "ball_obstacles");
	full_obstacles.add(self_obs);
	full_obstacles.add(opp_obs);
	full_obstacles.add(ball_obs);
	full_obstacles.add(global_obstacles);

	// sanity check the current position and escape if necessary
	ObstacleGroup hitset;
	if (full_obstacles.hit(pos, hitset) && !hitset.empty()) {
		addText(QString("execute: in obstacle"));
		if (verbose) cout << "Robot " << shell() << " inside " << hitset.size() << " obstacles, escaping" << endl;
		cmd.goalPosition = escapeObstacles(pos, hitset);
		cmd.pathLength = pos.distTo(cmd.goalPosition);
		cmd.planner = MotionCmd::Point;
		return;
	}

	// check for stopped condition
	// NOTE: after sanity check to ensure it will go to valid area first
	if (!_delayed_goal) {
		if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: stopped" << endl;
		addText(QString("execute: stopped"));
		cmd.goalPosition = pos;
		cmd.pathLength = 0;
		cmd.planner = MotionCmd::Point;
		return;
	}

	// create default path for comparison - swtich if available
	Planning::Path straight_line(pos, *_delayed_goal);
	Geometry2d::Segment straight_seg(pos, *_delayed_goal);
	if (!full_obstacles.hit(straight_seg)) {
		if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: using straight line goal" << endl;
		addText(QString("execute: straight_line"));
		_path = straight_line;
		cmd.goalPosition = *_delayed_goal;
		cmd.pathLength = straight_line.length(0);
		cmd.planner = MotionCmd::Point;
		drawPath(straight_line, Qt::red);
		return;
	}

	// create new a new path for comparision
	// TODO: somehow do this less often
	Planning::Path rrt_path = rrtReplan(*_delayed_goal, full_obstacles);
	const float rrt_path_len = rrt_path.length(pos);

	// check if goal is close to previous goal to reuse path
	Geometry2d::Point::Optional dest = _path.destination();
	if (dest && _delayed_goal->nearPoint(*dest, 0.1)) {

		// check if previous path is good enough to use - use if possible
		if (!_path.hit(full_obstacles) && _path.length(pos) > rrt_path_len + path_threshold) {
			if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: using previous path" << endl;
			addText(QString("execute: reusing path"));
			cmd.goalPosition = findGoalOnPath(pos, _path, full_obstacles);
			cmd.pathLength = _path.length(pos);
			cmd.planner = MotionCmd::Point;
			drawPath(_path, Qt::yellow);
			return;
		}
	}

	// use the newly generated path
	if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: using new RRT path" << endl;
	_path = rrt_path;
	drawPath(rrt_path, Qt::magenta);
	addText(QString("execute: RRT path %1").arg(full_obstacles.size()));
	cmd.goalPosition = findGoalOnPath(pos, _path, full_obstacles);
	cmd.pathLength = rrt_path_len;
	cmd.planner = MotionCmd::Point;
	return;

//	// Add ball obstacles
//	// NOTE: there will be a lag in the small obstacle avoidance due to planning execution order
//	// FIXME: removed small ball obstacle
//	if (verbose) cout << "  Adding ball obstacles" << endl;
//	if (visible && !isGoalie)	{
//		// Any robot that isn't the goalie may have to avoid the ball due to rules
//		if ((_state->gameState.state != GameState::Playing && !_state->gameState.ourRestart)) {// || avoidBall)
//			if (largeBallObstacle)
//				obstacles.add(largeBallObstacle);
//		}	else if (!willKick)	{
//			// Don't hit the ball unintentionally during normal play
//			if (smallBallObstacle)
//				obstacles.add(smallBallObstacle);
//		}
//	}

}
