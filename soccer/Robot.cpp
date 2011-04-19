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
	willKick = false;
	avoidBall = false;
	exclude = false;
	hasBall = false;
	avoidOpponents = true;
	sensorConfidence = 0;
	cmd_w = 0;
	_lastChargedTime = 0;

	_dynamics = new Planning::Dynamics(this);
	_planner = new Planning::RRT::Planner();

	for (size_t i = 0; i < Num_Shells; ++i)
	{
		approachOpponent[i] = false;
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
	// 	dbg->set_layer(_state->findDebugLayer(layer));
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

void OurRobot::resetMotionCommand()
{
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

	// Stay in place if possible.
	stop();
}

void OurRobot::stop()
{
	move(pos);
}

void OurRobot::move(const OptionalPoint& goal, const MoveType& goal_param,
		const OptionalPoint& facing, const FacingType& facing_param) {

}

void OurRobot::avoid(const BallAvoid& ball, const RobotMask& opp_robots,
		const RobotMask& self_robots, const ObstacleGroup& regions) {

}

void OurRobot::move(Geometry2d::Point goal, bool stopAtEnd)
{
	if (!visible)
		return;

//	// set planning target and continue
//	// Rest of planning will occur after plays are complete
//	if (_planner_goal && _planner_goal)
//	_planner_goal = pt;
//	_planner_stopAtEnd = stopAtEnd;
//
//	// create a new path
//	Planning::Path newPath;
//
//	// determine the obstacles
//	ObstacleGroup& og = obstacles;
//
//	// run the RRT planner to generate a new plan
//	_planner->run(pos, angle, vel, pt, &og, newPath);
//
//	//Without this soccer seg faults when its started (I don't know why - Anthony)
//	if(_path.empty() ||
//			(_path.destination().distTo(newPath.destination()) > Robot_Radius) ||
//			(_path.hit(og, 0, false)) ||
//			(_path.length(0) > newPath.length(0) + path_threshold))
//	{
//		_path = newPath;
//
//		Geometry2d::Point last = pos;
//		BOOST_FOREACH(Geometry2d::Point pt, _path.points)
//		{
//			_state->drawLine(last, pt);
//			last = pt;
//		}
//	}

	// call the path move command
	executeMove(stopAtEnd);
}

void OurRobot::move(const vector<Geometry2d::Point>& path, bool stopAtEnd)
{
	_state->drawLine(path.back(), pos);

	// copy path from input
	_path.clear();
	_path.points = path;

	// execute
	executeMove(stopAtEnd);
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

void OurRobot::executeMove(bool stopAtEnd) // FIXME: need to do something with stopAtEnd
{
	setCommandTrace();

	// given a path, determine what the best point to use as a
	// target point is, and assign to packet

	if (_path.empty()) {
		// to avoid crashing if the path is empty, just stop the robot
		stop();
		return;
	}

	//dynamics path
	float length = _path.length();

	// handle direct point commands where the length may be very small
	if (fabs(length) < 1e-5) {
		length = pos.distTo(_path.points[0]);
	}

	//target point is the last point on the closest segment
	if (_path.points.size() == 1) {
		cmd.goalPosition = _path.points[0]; // first point
		cmd.pathLength = pos.distTo(cmd.goalPosition);
		cmd.planner = MotionCmd::Point;
		return;
	}
	// simple case: one segment, go to end of segment
	else if (_path.points.size() == 2)
	{
		cmd.goalPosition = _path.points[1];
		cmd.pathLength = _path.length(0);
		cmd.planner = MotionCmd::Point;
		return;
	}

	// All other cases: mix the next two points together for a smoother
	// path, so long as it is still viable

	// pull out relevant points
	Point p0 = pos, p1 = _path.points[1], p2 = _path.points[2];
	float dist1 = p1.distTo(p1), dist2 = p1.distTo(p2);

	// mix the next point between the first and second point
	float scale = 1-Utils::clamp(dist1/dist2, 1.0, 0.0);
	Geometry2d::Point targetPos = p1 + (p2-p1)*scale;

	// check for collisions
	Planning::Path smoothPath;
	smoothPath.points.push_back(p0);
	smoothPath.points.push_back(targetPos);
	if (smoothPath.hit(obstacles, 0, false)) {
		// go to original next state
		cmd.goalPosition = p1;
		cmd.pathLength = _path.length(0);

	} else {
		cmd.goalPosition = targetPos;
		cmd.pathLength = pos.distTo(targetPos) + targetPos.distTo(p2) + _path.length(2);
	}
	cmd.planner = MotionCmd::Point;
	return;
}

void OurRobot::directVelocityCommands(const Geometry2d::Point& trans, double ang)
{
	cmd.planner = MotionCmd::DirectVelocity;
	cmd.direct_ang_vel = ang;
	cmd.direct_trans_vel = trans;
}

void OurRobot::directMotorCommands(const vector<int8_t>& speeds) {
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
	willKick = true;
	radioTx.set_kick(strength);
	radioTx.set_use_chipper(false);
}

void OurRobot::chip(uint8_t strength)
{
	willKick = true;
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

void OurRobot::execute(const ObstacleGroup& global_obstacles, const ObstacleGroup& goal_area, bool isGoalie) {
	const bool verbose = false;

	// determine obstacles
	ObstaclePtr largeBallObstacle;
	ObstaclePtr smallBallObstacle;
	if (_state->ball.valid)	{
		largeBallObstacle = ObstaclePtr(new CircleObstacle(_state->ball.pos, Field_CenterRadius));
		smallBallObstacle = ObstaclePtr(new CircleObstacle(_state->ball.pos, Ball_Radius));
	}

	ObstaclePtr selfObstacles[Num_Shells];
	ObstaclePtr oppObstacles[Num_Shells];
	for (unsigned int i = 0; i < Num_Shells; ++i)	{
		if (_state->self[i]->visible)
			selfObstacles[i] = ObstaclePtr(new CircleObstacle(_state->self[i]->pos, Robot_Radius - .01));
		if (_state->opp[i]->visible) {
			// FIXME: find a better size of obstacle avoidance obstacles - old radius was: Robot_Radius - .01
			const float oppAvoidRadius = Robot_Radius - 0.03;
			oppObstacles[i] = ObstaclePtr((new CircleObstacle(_state->opp[i]->pos, oppAvoidRadius)));
		}
	}

	// Reset the motion command
	// FIXME: this also resets flags from the previous frame
	resetMotionCommand();

	if (visible) {
		// Add obstacles for this robot
		obstacles.clear();

		// Add rule-based obstacles (except for the ball, which will be added after the play
		// has a chance to set willKick and avoidBall)
		// NOTE: this uses the avoidOpponents flag from the last frame to set this
		for (unsigned int i = 0; i < Num_Shells; ++i)		{
			if (i != shell() && selfObstacles[i])
				obstacles.add(selfObstacles[i]);
			if (!approachOpponent[i] && avoidOpponents && oppObstacles[i])
				obstacles.add(oppObstacles[i]);
		}

		//if not a goalie, avoid our goalie area
		if (!isGoalie)
			obstacles.add(goal_area);
	}

	// Add ball obstacles
	// NOTE: there will be a lag in the small obstacle avoidance due to planning execution order
	// FIXME: removed small ball obstacle
	if (verbose) cout << "  Adding ball obstacles" << endl;
	if (visible && !isGoalie)	{
		// Any robot that isn't the goalie may have to avoid the ball due to rules
		if ((_state->gameState.state != GameState::Playing && !_state->gameState.ourRestart)) {// || avoidBall)
			if (largeBallObstacle)
				obstacles.add(largeBallObstacle);
		}	else if (!willKick)	{
			// Don't hit the ball unintentionally during normal play
			if (smallBallObstacle)
				obstacles.add(smallBallObstacle);
		}
	}

	// perform planning

	// construct motion command
}
