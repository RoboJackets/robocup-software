#include <Robot.hpp>
#include <gameplay/planning/bezier.hpp>
#include <Utils.hpp>
#include <LogUtils.hpp>
#include <motion/MotionControl.hpp>
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
//const float path_threshold = 2 * Robot_Diameter; // previous value
const float path_threshold = 1.0;

// thresholds for avoidance of opponents - either a normal (large) or an approach (small)
const float Opp_Avoid_Small = Robot_Radius - 0.03;
const float Opp_Avoid_Large = Robot_Radius - 0.01;

const float Ball_Avoid_Small = 2.0 * Ball_Radius;

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
	_ball_avoid = Ball_Avoid_Small;
	_delayed_goal = boost::none;
	_planner_type = RRT;
	exclude = false;
	hasBall = false;
	sensorConfidence = 0;
	cmd_w = 0;
	_lastChargedTime = 0;
	_motionControl = new MotionControl(this);
	_stopAtEnd = false;

	_planner = new Planning::RRT::Planner();
	for (size_t i = 0; i < Num_Shells; ++i)
	{
		// TODO move thresholds elsewhere
		_self_avoid_mask[i] = (i != (size_t) shell) ? Robot_Radius : -1.0;
		_opp_avoid_mask[i] = Opp_Avoid_Large;
	}

	_planner->maxIterations(250);
}

OurRobot::~OurRobot()
{
	delete _motionControl;
	delete _planner;
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
	BOOST_FOREACH(float &a, _opp_avoid_mask)
		if (enable)
			a = Robot_Radius - 0.03;
		else
			a = -1.0;
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

	cmd = MotionCommand();

	_local_obstacles.clear();

}

void OurRobot::stop()
{
	_delayed_goal = boost::none;
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
	_stopAtEnd = stopAtEnd;
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
	cmd.target = MotionTarget();
	cmd.target->pos = findGoalOnPath(pos, _path);
	cmd.target->pathLength = _path.length(pos);
	cmd.target->pathEnd = (stopAtEnd) ? MotionTarget::StopAtEnd : MotionTarget::FastAtEnd;
}

void OurRobot::pivot(double w, double radius)
{
	directVelocityCommands(Point(0, -radius * w), w);
}

void OurRobot::directVelocityCommands(const Geometry2d::Point& trans, double ang)
{
	// ensure RRT not used
	_delayed_goal = boost::none;
	_planner_type = OVERRIDE;

	cmd.target = boost::none;
	cmd.worldVel = boost::none;
	cmd.bodyVel = trans;
	cmd.angularVelocity = ang;
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

bool OurRobot::hasChipper() const
{
	return false;
}

void OurRobot::dribble(int8_t speed)
{
	radioTx.set_roller(speed);
}

void OurRobot::face(Geometry2d::Point pt, bool continuous)
{
	cmd.face = FaceTarget();
	cmd.face->pos = pt;
	cmd.face->continuous = continuous;
}

void OurRobot::faceNone()
{
	cmd.face = boost::none;
}

void OurRobot::kick(uint8_t strength)
{
	radioTx.set_kick(strength);
	radioTx.set_use_chipper(false);
}

void OurRobot::chip(uint8_t strength)
{
	radioTx.set_kick(strength);
	radioTx.set_use_chipper(true);
}

bool OurRobot::charged() const
{
	return radioRx.charged();
}

void OurRobot::approachAllOpponents(bool enable) {
	BOOST_FOREACH(float &ar, _opp_avoid_mask)
		ar = (enable) ?  Opp_Avoid_Small : Opp_Avoid_Large;
}
void OurRobot::avoidAllOpponents(bool enable) {
	BOOST_FOREACH(float &ar, _opp_avoid_mask)
		ar = (enable) ?  -1.0 : Opp_Avoid_Large;
}

bool OurRobot::avoidOpponent(unsigned shell_id) const {
	return _opp_avoid_mask[shell_id] > 0.0;
}

bool OurRobot::approachOpponent(unsigned shell_id) const {
	return avoidOpponent(shell_id) && _opp_avoid_mask[shell_id] < Robot_Radius - 0.01;
}

float OurRobot::avoidOpponentRadius(unsigned shell_id) const {
	return _opp_avoid_mask[shell_id];
}

void OurRobot::avoidOpponent(unsigned shell_id, bool enable_avoid) {
	if (enable_avoid)
		_opp_avoid_mask[shell_id] = Opp_Avoid_Large;
	else
		_opp_avoid_mask[shell_id] = -1.0;
}

void OurRobot::approachOpponent(unsigned shell_id, bool enable_approach) {
	if (enable_approach)
		_opp_avoid_mask[shell_id] = Opp_Avoid_Small;
	else
		_opp_avoid_mask[shell_id] = Opp_Avoid_Large;
}

void OurRobot::avoidOpponentRadius(unsigned shell_id, float radius) {
	_opp_avoid_mask[shell_id] = radius;
}

void OurRobot::avoidAllTeammates(bool enable) {
	for (size_t i=0; i<Num_Shells; ++i)
		avoidTeammate(i, enable);
}
void OurRobot::avoidTeammate(unsigned shell_id, bool enable) {
	if (shell_id != shell())
		_self_avoid_mask[shell_id] = (enable) ? Robot_Radius : -1.0;
}

void OurRobot::avoidTeammateRadius(unsigned shell_id, float radius) {
	if (shell_id != shell())
		_self_avoid_mask[shell_id] = radius;
}

bool OurRobot::avoidTeammate(unsigned shell_id) const {
	return _self_avoid_mask[shell_id] < Robot_Radius;
}

float OurRobot::avoidTeammateRadius(unsigned shell_id) const {
	return _self_avoid_mask[shell_id];
}

void OurRobot::disableAvoidBall() {
	_ball_avoid = -1.0;
}

void OurRobot::avoidBall(float radius) {
	_ball_avoid = radius;
}

float OurRobot::avoidBall() const {
	return _ball_avoid;
}

ObstaclePtr OurRobot::createBallObstacle() const {
	// if game is stopped, large obstacle regardless of flags
	if (_state->gameState.state != GameState::Playing && !_state->gameState.ourRestart)
		return ObstaclePtr(new CircleObstacle(_state->ball.pos, Field_CenterRadius));

	// create an obstacle if necessary
	if (_ball_avoid > 0.0) {
		return ObstaclePtr(new CircleObstacle(_state->ball.pos, _ball_avoid));
	} else {
		return ObstaclePtr();
	}
}

Geometry2d::Point OurRobot::findGoalOnPath(const Geometry2d::Point& pose,
		const Planning::Path& path,	const ObstacleGroup& obstacles) {
		const bool blend_verbose = false;
// 		setCommandTrace();

		// empty path case - leave robot stationary
		if (path.empty())
			return pose;

		// go to nearest point if only point or closest point is the goal
		if (path.size() == 1) {
			if (blend_verbose) addText(QString("blend:simple_path"));
			return path.points[0];
		}

		// can't mix, just go to endpoint
		if (path.size() == 2) {
			if (blend_verbose) addText(QString("blend:size2"));
			return path.points[1];
		}

		// FIXME: this doesn't work well - gets stuck in some places
		// All other cases: proportionally blend the next two points together for a smoother
		// path, so long as it is still viable

		if (blend_verbose) addText(QString("blend:segments=%1").arg(path.points.size()-1));

		// pull out relevant points
		Point p0 = pos,
				  p1 = path.points[1],
				  p2 = path.points[2];
		Geometry2d::Segment target_seg(p1, p2);

		// final endpoint handling
		if (target_seg.nearPointPerp(p0, 0.02) && p0.nearPoint(p2, 0.03)) {
			if (2 == path.size()-1) {
				if (blend_verbose) addText(QString("blend:at_end"));
				return p2;
			} else {
				// reset this segment to next one
				if (blend_verbose) addText(QString("blend:reset_segment"));
			  p1 = path.points[2],
			  p2 = path.points[1];
			  target_seg = Geometry2d::Segment(p1, p2);
			}
		}

		float dist1 = p0.distTo(p1), dist2 = p1.distTo(p2);
		if (blend_verbose) addText(QString("blend:d1=%1,d2=%2").arg(dist1).arg(dist2));

		// endpoint handling
		if (dist1 < 0.02) {
			if (blend_verbose) addText(QString("blend:dist1small=%1").arg(dist1));
			return p2; /// just go to next point
		}

		// short segment handling
		if (p1.distTo(p2) < 0.05) {
			if (blend_verbose) addText(QString("blend:dist2small=%1").arg(dist2));
			return p2; /// just go to next point
		}

		// close to segment - go to end of segment
		if (target_seg.nearPoint(p0, 0.03)) {
			if (blend_verbose) addText(QString("blend:closeToSegment"));
			return p2;
		}

		// mix the next point between the first and second point
		// if we are far away from p1, want scale to be closer to p1
		// if we are close to p1, want scale to be closer to p2
		float scale = 1-Utils::clamp(dist1/dist2, 1.0, 0.0);
		Geometry2d::Point targetPos = p1 + (p2-p1)*scale;
		if (blend_verbose) {
			addText(QString("blend:scale=%1").arg(scale));
			addText(QString("blend:dist1=%1").arg(dist1));
			addText(QString("blend:dist2=%1").arg(dist2));
		}

		// check for collisions on blended path
		Geometry2d::Segment shortcut(p0, targetPos);

		Geometry2d::Point result = p1;
		if (!obstacles.hit(shortcut)) {
			if (blend_verbose) addText(QString("blend:shortcut_succeed"));
			result = targetPos;
		} else if (result.nearPoint(pose, 0.05)) {
			if (blend_verbose) addText(QString("blend:shortcut_failed"));
			result = result + (result-pose).normalized() * 0.10;
		}

		if (blend_verbose) addText(QString("point (%1, %2)").arg(result.x).arg(result.y));

		return result;
}

Planning::Path OurRobot::rrtReplan(const Geometry2d::Point& goal,
		const ObstacleGroup& obstacles) {
// 	setCommandTrace();

	// create a new path
	Planning::Path result;

	// run the RRT planner to generate a new plan
	_planner->run(pos, angle, vel, goal, &obstacles, result);

	return result;
}

void OurRobot::drawPath(const Planning::Path& path, const QColor &color, const QString &layer) {
	Geometry2d::Point last = pos;
	BOOST_FOREACH(Geometry2d::Point pt, path.points)
	{
		_state->drawLine(last, pt, color, layer);
		last = pt;
	}
}

void OurRobot::execute(const ObstacleGroup& global_obstacles) {
// 	setCommandTrace();

	const bool enable_slice = false;

	// halt case - same as stopped
	if (_state->gameState.state == GameState::Halt) {
		return;
	}

	// if motion command complete or we are using a different planner - we're done
	if (_planner_type != OurRobot::RRT) {
		if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: non-RRT planner" << endl;
		return;
	}

	cmd.target = MotionTarget();
	cmd.target->pathEnd = (_stopAtEnd) ? MotionTarget::StopAtEnd : MotionTarget::FastAtEnd;
	
	// create and visualize obstacles
	ObstacleGroup full_obstacles(_local_obstacles);
	ObstacleGroup
		self_obs = createRobotObstacles(_state->self, _self_avoid_mask),
		opp_obs = createRobotObstacles(_state->opp, _opp_avoid_mask);
	ObstaclePtr ball_obs = createBallObstacle();
	_state->drawObstacles(self_obs, Qt::gray, QString("self_obstacles_%1").arg(shell()));
	_state->drawObstacles(opp_obs, Qt::gray, QString("opp_obstacles_%1").arg(shell()));
	_state->drawObstacle(ball_obs, Qt::gray, QString("ball_obstacles_%1").arg(shell()));
	full_obstacles.add(self_obs);
	full_obstacles.add(opp_obs);
	full_obstacles.add(ball_obs);
	full_obstacles.add(global_obstacles);

	// if no goal command robot to stop in place
	if (!_delayed_goal) {
		if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: stopped" << endl;
		addText(QString("execute: no goal"));
		_path = Planning::Path(pos);
		cmd.target->pos = pos;
		cmd.target->pathLength = 0;
		drawPath(_path);
		return;
	}

	// create default path for comparison - switch if available
	Planning::Path straight_line(pos, *_delayed_goal);
	Geometry2d::Segment straight_seg(pos, *_delayed_goal);
	if (!full_obstacles.hit(straight_seg)) {
		if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: using straight line goal" << endl;
		addText(QString("execute: straight_line"));
		_path = straight_line;
		cmd.target->pos = *_delayed_goal;
		cmd.target->pathLength = straight_line.length(0);
		drawPath(straight_line, Qt::red);
		return;
	}

	// create new a new path for comparision
	Planning::Path rrt_path = rrtReplan(*_delayed_goal, full_obstacles);
	const float rrt_path_len = rrt_path.length(0);

	// check if goal is close to previous goal to reuse path
	Geometry2d::Point::Optional dest = _path.destination();
	if (dest && _delayed_goal->nearPoint(*dest, 0.1)) {
		Planning::Path sliced_path;
		_path.startFrom(pos, sliced_path);
		if (enable_slice && !sliced_path.hit(full_obstacles)) {
			addText(QString("execute: slicing path"));
			cmd.target->pos = findGoalOnPath(pos, sliced_path, full_obstacles);
			cmd.target->pathLength = sliced_path.length(pos);
			drawPath(sliced_path, Qt::cyan);
			Geometry2d::Point offset(0.01, 0.01);
			_state->drawLine(pos + offset, cmd.target->pos + offset, Qt::black);
			return;
		} else if (!_path.hit(full_obstacles)) {
			addText(QString("execute: reusing path"));
			cmd.target->pos = findGoalOnPath(pos, _path, full_obstacles);
			cmd.target->pathLength = _path.length(pos);
			drawPath(_path, Qt::yellow);
			_state->drawLine(pos, cmd.target->pos, Qt::black);
			return;
		}
	}

	// use the newly generated path
	if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: using new RRT path" << endl;
	_path = rrt_path;
	drawPath(rrt_path, Qt::magenta);
	addText(QString("execute: RRT path %1").arg(full_obstacles.size()));
	cmd.target->pos = findGoalOnPath(pos, _path, full_obstacles);
	cmd.target->pathLength = rrt_path_len;
	return;
}
