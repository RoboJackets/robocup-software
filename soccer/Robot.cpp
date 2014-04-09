#include <Robot.hpp>
#include <Utils.hpp>
#include <LogUtils.hpp>
#include <motion/MotionControl.hpp>
#include <protobuf/LogFrame.pb.h>
#include <SystemState.hpp>
#include <RobotConfig.hpp>
#include <modeling/RobotFilter.hpp>

#include <stdio.h>
#include <iostream>
#include <execinfo.h>
#include <stdexcept>
#include <boost/foreach.hpp>
#include <QString>

using namespace std;
using namespace Geometry2d;

/** Constant for timestamp to seconds */
const float intTimeStampToFloat = 1000000.0f;

///The threshold necessary to change paths tuning required
///const float path_threshold = 2 * Robot_Diameter; // previous value
const float path_threshold = 1.0;

/** thresholds for avoidance of opponents - either a normal (large) or an approach (small)*/
const float Opp_Avoid_Small = Robot_Radius - 0.03;
/** thresholds for avoidance of opponents - either a normal (large) or an approach (small)*/
const float Opp_Avoid_Large = Robot_Radius - 0.01;
/** threshold for avoiding the ball*/
const float Ball_Avoid_Small = 2.0 * Ball_Radius;
/**
 * I think this is verbose comments in the simulator
 */
const bool verbose = false;

/**
 * super class for all robots
 *@param shell this is a robot id number
 *@param self is robot on our team?
 */
Robot::Robot(unsigned int shell, bool self)
{
	visible = false;
	_shell = shell;
	_self = self;
	angle = 0;
	angleVel = 0;
	
	_filter = new RobotFilter();
}
/**
 * deconstructor, deletesRobotFilter
 */
Robot::~Robot()
{
	delete _filter;
	_filter = 0;
}


#pragma mark OurRobot

OurRobot::OurRobot(int shell, SystemState *state):
	Robot(shell, true),
	_state(state)
{
	resetAvoidBall();
	_lastChargedTime = 0;
	_lastKickerStatus = 0;
	_lastKickTime = 0;

	_motionControl = new MotionControl(this);

	_planner = new Planning::RRTPlanner();
	_planner->maxIterations(250);

	for (size_t i = 0; i < Num_Shells; ++i)
	{
		// TODO move thresholds elsewhere
		_self_avoid_mask[i] = (i != (size_t) shell) ? Robot_Radius : -1.0;
		_opp_avoid_mask[i] = Opp_Avoid_Large;
	}

}

/**
 * ourrobot deconstructor, deletes motion control and planner
 */
OurRobot::~OurRobot()
{
	delete _motionControl;
	delete _planner;
}

void OurRobot::addStatusText()
{
	static const char *motorNames[] = {"BL", "FL", "FR", "BR", "DR"};
	
	const QColor statusColor(255, 32, 32);
	
	if (!rxIsFresh())
	{
		addText("No RX", statusColor, "Status");
		
		// No more status is available
		return;
	}
	
	// Motor status
	if (_radioRx.motor_status().size() == 5)
	{
		for (int i = 0; i < 5; ++i)
		{
			QString error;
			switch (_radioRx.motor_status(i))
			{
				case Packet::Hall_Failure:
					error = "Hall fault";
					break;
				
				case Packet::Stalled:
					error = "Stall";
					break;
				
				case Packet::Encoder_Failure:
					error = "Encoder fault";
					break;
				
				default:
					break;
			}
			
			if (!error.isNull())
			{
				addText(QString("%1: %2").arg(error, QString(motorNames[i])), statusColor, "Status");
			}
		}
	}
	
	if (!ballSenseWorks())
	{
		addText("Ball sense fault", statusColor, "Status");
	}
	
	if (!kickerWorks() && false)
	{
		addText("Kicker fault", statusColor, "Status");
	}
	
	if (_radioRx.has_battery())
	{
		float battery = _radioRx.battery();
		if (battery <= 14.3f)
		{
			addText(QString("Low battery: %1V").arg(battery, 0, 'f', 1), statusColor, "Status");
		}
	}
}

void OurRobot::addText(const QString& text, const QColor& qc, const QString &layerPrefix)
{
	Packet::DebugText *dbg = new Packet::DebugText;
	QString layer = layerPrefix + QString::number(shell());
	dbg->set_layer(_state->findDebugLayer(layer));
	dbg->set_text(text.toStdString());
	dbg->set_color(color(qc));
	robotText.push_back(dbg);
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

void OurRobot::resetForNextIteration() {
	if (verbose && visible) cout << "in OurRobot::resetMotionCommand()" << endl;
	robotText.clear();	//	FIXME: this doesn't belong here, but it was in the previous version of this method
	
	radioTx.Clear();
	radioTx.set_robot_id(shell());
	radioTx.set_accel(10);
	radioTx.set_decel(10);

	_local_obstacles.clear();
}

void OurRobot::resetMotionConstraints() {
	_motionConstraints = MotionConstraints();
}

void OurRobot::stop() {
	resetMotionConstraints();
}

void OurRobot::move(const Geometry2d::Point &goal, bool stopAtEnd)
{
	if (!visible)
		return;

	// sets flags for future movement
	if (verbose) cout << " in OurRobot::move(goal): adding a goal (" << goal.x << ", " << goal.y << ")" << endl;
	addText(QString("move:(%1, %2)").arg(goal.x).arg(goal.y));
	
	//	only invalidate path if move() is being called with a new goal or one wasn't set previously
	if (!_motionConstraints.targetPos || !_motionConstraints.targetPos->nearPoint(goal, 0.02)) {
		addText("Invalidated old path");
		
		if (_motionConstraints.targetPos) {
			addText(QString("Old goal: (%1, %2)").arg(goal.x, goal.y));
			addText(QString("New goal: (%1, %2)").arg(_motionConstraints.targetPos->x, _motionConstraints.targetPos->y));
		} else {
			addText("Old goal was null");
		}
		
		_motionConstraints.targetPos = goal;
		_pathInvalidated = true;
	}
}

// void OurRobot::pivot(double w, double radius)
// {
// 	bodyVelocity(Point(0, -radius * w));
// 	angularVelocity(w);
// }

// void OurRobot::bodyVelocity(const Geometry2d::Point& v)
// {

// 	// ensure RRT not used
// 	_delayed_goal = boost::none;
// 	_usesPathPlanning = false;

// 	cmd.target = boost::none;
// 	cmd.worldVel = boost::none;
// 	cmd.bodyVel = v;
// }

void OurRobot::worldVelocity(const Geometry2d::Point& v)
{
	_motionConstraints.targetPos = boost::none;
	_motionConstraints.targetWorldVel = v;

	//	FIXME: clear Path?
}

// void OurRobot::angularVelocity(double w)
// {
// 	cmd.angularVelocity = w;
// }

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

float OurRobot::kickTimer() const {
	return (charged()) ? 0.0 : intTimeStampToFloat * (float) (timestamp() - _lastChargedTime);
}

void OurRobot::update() {
	if (charged())
	{
		_lastChargedTime = timestamp();
	}
}

void OurRobot::dribble(int8_t speed)
{
	radioTx.set_dribbler(speed);
}

void OurRobot::face(const Geometry2d::Point &pt)
{
	_motionConstraints.faceTarget = pt;
}

void OurRobot::faceNone()
{
	_motionConstraints.faceTarget = boost::none;
}

void OurRobot::kick(uint8_t strength)
{
	uint8_t max = *config->kicker.maxKick;
	radioTx.set_kick(strength > max ? max : strength);
	radioTx.set_use_chipper(false);
}

void OurRobot::chip(uint8_t strength)
{
	uint8_t max = *config->kicker.maxChip;
	radioTx.set_kick(strength > max ? max : strength);
	radioTx.set_use_chipper(true);
}

void OurRobot::immediate(bool im)
{
	radioTx.set_kick_immediate(im);
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

void OurRobot::avoidAllOpponentRadius(float radius) {
	BOOST_FOREACH(float &ar, _opp_avoid_mask)
		ar = radius;
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


#pragma mark Ball Avoidance

void OurRobot::disableAvoidBall() {
	_avoidBallRadius = -1.0;
}

void OurRobot::avoidBallRadius(float radius) {
	_avoidBallRadius = radius;
}

float OurRobot::avoidBallRadius() const {
	return _avoidBallRadius;
}

void OurRobot::resetAvoidBall() {
	avoidBallRadius(Ball_Avoid_Small);
}

ObstaclePtr OurRobot::createBallObstacle() const {
	// if game is stopped, large obstacle regardless of flags
	if (_state->gameState.state != GameState::Playing && !(_state->gameState.ourRestart || _state->gameState.theirPenalty()))
	{
		return ObstaclePtr(new CircleObstacle(_state->ball.pos, Field_CenterRadius));
	}

	// create an obstacle if necessary
	if (_avoidBallRadius > 0.0) {
		return ObstaclePtr(new CircleObstacle(_state->ball.pos, _avoidBallRadius));
	} else {
		return ObstaclePtr();
	}
}


#pragma mark Motion

void OurRobot::setPath(Planning::Path path) {
	_path = path;
	_pathInvalidated = false;
	_pathStartTime = timestamp();
	_path->setStartSpeed(vel.mag());
}

void OurRobot::replanIfNeeded(const ObstacleGroup& global_obstacles) {
	// halt case - same as stopped
	if (_state->gameState.state == GameState::Halt) {
		return;
	}

	// if motion command complete or we are using a different planner - we're done
	if (!_motionConstraints.targetPos) {
		if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: not using path planner" << endl;
		return;
	}
	
	// create and visualize obstacles
	ObstacleGroup full_obstacles(_local_obstacles);
	ObstacleGroup
		self_obs = createRobotObstacles(_state->self, _self_avoid_mask),
		opp_obs = createRobotObstacles(_state->opp, _opp_avoid_mask);
	_state->drawObstacles(self_obs, Qt::gray, QString("self_obstacles_%1").arg(shell()));
	_state->drawObstacles(opp_obs, Qt::gray, QString("opp_obstacles_%1").arg(shell()));
	if (_state->ball.valid)
	{
		ObstaclePtr ball_obs = createBallObstacle();
		_state->drawObstacle(ball_obs, Qt::gray, QString("ball_obstacles_%1").arg(shell()));
		full_obstacles.add(ball_obs);
	}
	full_obstacles.add(self_obs);
	full_obstacles.add(opp_obs);
	full_obstacles.add(global_obstacles);

	// if no goal command robot to stop in place
	if (!_motionConstraints.targetPos) {
		if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: stopped" << endl;
		addText(QString("execute: no goal"));
		setPath(Planning::Path(pos));
		_state->drawPath(*_path);
		return;
	}


	Geometry2d::Point dest = *_motionConstraints.targetPos;

	//	if this number of microseconds passes since our last path plan, we automatically replan
	const uint64_t kPathExpirationInterval = 1500000;	//	1.5 seconds
	if ((timestamp() - _pathStartTime) > kPathExpirationInterval) {
		_pathInvalidated = true;
	}

	//	invalidate path if it hits obstacles
	if (_path && _path->hit(full_obstacles)) {
		_pathInvalidated = true;
	}

	//	if the destination of the current path is greater than 1cm away from the target destination,
	//	we invalidate the path.  this situation could arise if during a previous planning, the target point
	//	was blocked by an obstacle
	if (_path && (_path->points.back() - dest).mag() > 0.01) {
		_pathInvalidated = true;
	}

	if (!_path) {
		_pathInvalidated = true;
	}

	// check if goal is close to previous goal to reuse path
	if (!_pathInvalidated) {
		addText("Reusing path");
		Planning::Path sliced_path;
		_path->startFrom(pos, sliced_path);

		const bool enable_slice = false;
		if (enable_slice && !sliced_path.hit(full_obstacles)) {
			addText(QString("execute: slicing path"));
			_state->drawPath(sliced_path, Qt::cyan);
			return;
		}
	} else {
		// use the newly generated path
		if (verbose) cout << "in OurRobot::execute() for robot [" << shell() << "]: using new RRT path" << endl;
		
		// create new a new path
		Planning::Path newlyPlannedPath;
		_planner->run(pos, angle, vel, *_motionConstraints.targetPos, &full_obstacles, newlyPlannedPath);
		setPath(newlyPlannedPath);
	}

	_state->drawPath(*_path, Qt::magenta);
	addText(QString("execute: RRT path %1").arg(full_obstacles.size()));
	return;
}

bool OurRobot::charged() const
{
	return _radioRx.has_kicker_status() && (_radioRx.kicker_status() & 0x01) && rxIsFresh();
}

bool OurRobot::hasBall() const
{
	return _radioRx.has_ball_sense_status() && _radioRx.ball_sense_status() == Packet::HasBall && rxIsFresh();
}

bool OurRobot::ballSenseWorks() const
{
	return rxIsFresh() && _radioRx.has_ball_sense_status() && (_radioRx.ball_sense_status() == Packet::NoBall || _radioRx.ball_sense_status() == Packet::HasBall);
}

bool OurRobot::kickerWorks() const
{
	return _radioRx.has_kicker_status() && !(_radioRx.kicker_status() & 0x80) && rxIsFresh();
}

bool OurRobot::chipper_available() const
{
	return hardwareVersion() == Packet::RJ2011 && kickerWorks() && *status->chipper_enabled;
}

bool OurRobot::kicker_available() const
{
	return kickerWorks() && *status->kicker_enabled;
}

bool OurRobot::dribbler_available() const {
	return *status->dribbler_enabled && _radioRx.motor_status_size() == 5 && _radioRx.motor_status(4) == Packet::Good;
}

bool OurRobot::driving_available(bool require_all) const
{
	if (_radioRx.motor_status_size() != 5)
		return false;
	int c = 0;
	for (int i=0; i<4; ++i)
	{
		if (_radioRx.motor_status(i) == Packet::Good)
		{
			++c;
		}
	}
	return (require_all) ? c == 4 : c == 3;
}

float OurRobot::kickerVoltage() const
{
	if (_radioRx.has_kicker_voltage() && rxIsFresh())
	{
		return _radioRx.kicker_voltage();
	} else {
		return 0;
	}
}

Packet::HardwareVersion OurRobot::hardwareVersion() const
{
	if (rxIsFresh())
	{
		return _radioRx.hardware_version();
	} else {
		return Packet::Unknown;
	}
}

boost::optional<Eigen::Quaternionf> OurRobot::quaternion() const
{
	if (_radioRx.has_quaternion() && rxIsFresh(50000))
	{
		return Eigen::Quaternionf(
			_radioRx.quaternion().q0() / 16384.0,
			_radioRx.quaternion().q1() / 16384.0,
			_radioRx.quaternion().q2() / 16384.0,
			_radioRx.quaternion().q3() / 16384.0);
	} else {
		return boost::none;
	}
}

bool OurRobot::rxIsFresh(uint64_t age) const
{
	return (timestamp() - _radioRx.timestamp()) < age;
}




uint64_t OurRobot::lastKickTime() const {
	return _lastKickTime;
}


void OurRobot::setRadioRx(Packet::RadioRx rx) {
	_radioRx = rx;

	if ( rx.kicker_status() < _lastKickerStatus ) {
		_lastKickTime = timestamp();
	}
	_lastKickerStatus = rx.kicker_status();
	//	FIXME: implement
}

