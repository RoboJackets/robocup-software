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
#include <cmath>

using namespace std;
using namespace Geometry2d;

/** thresholds for avoidance of opponents - either a normal (large) or an approach (small)*/
const float Opp_Avoid_Small = Robot_Radius - 0.03;
/** threshold for avoiding the ball*/
const float Ball_Avoid_Small = 2.0 * Ball_Radius;
/**
 * When verbose is true, a lot of extra debug info is printed
 * to the console about path planning, etc
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
	_filter = nullptr;
}


#pragma mark OurRobot

REGISTER_CONFIGURABLE(OurRobot)

ConfigDouble *OurRobot::_selfAvoidRadius;
ConfigDouble *OurRobot::_oppAvoidRadius;
ConfigDouble *OurRobot::_oppGoalieAvoidRadius;

void OurRobot::createConfiguration(Configuration *cfg) {
	_selfAvoidRadius = new ConfigDouble(cfg, "PathPlanner/selfAvoidRadius", Robot_Radius);
	_oppAvoidRadius = new ConfigDouble(cfg, "PathPlanner/oppAvoidRadius", Robot_Radius - 0.01);
	_oppGoalieAvoidRadius = new ConfigDouble(cfg, "PathPlanner/oppGoalieAvoidRadius", Robot_Radius + 0.05);
}

OurRobot::OurRobot(int shell, SystemState *state):
	Robot(shell, true),
	_state(state),
	_pathChangeHistory(PathChangeHistoryBufferSize)
{
	_cmdText = new std::stringstream();

	resetAvoidBall();
	_lastChargedTime = 0;
	_lastKickerStatus = 0;
	_lastKickTime = 0;

	_motionControl = new MotionControl(this);

	_planner = new Planning::RRTPlanner();
	_planner->maxIterations(250);

	resetAvoidRobotRadii();

	_clearCmdText();
}

/**
 * ourrobot deconstructor, deletes motion control and planner
 */
OurRobot::~OurRobot()
{
	if (_motionControl) delete _motionControl;
	if (_planner) delete _planner;
	delete _cmdText;
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

std::string OurRobot::getCmdText() const {
	return _cmdText->str();
}

void OurRobot::_clearCmdText() {
	_cmdText->str("");
	_cmdText->clear();
}

void OurRobot::resetForNextIteration() {
	if (verbose && visible) cout << "in OurRobot::resetForNextIteration()" << std::endl;
	robotText.clear();

	_didSetPathThisIteration = false;

	_clearCmdText();

	radioTx.Clear();
	radioTx.set_robot_id(shell());
	radioTx.set_accel(10);
	radioTx.set_decel(10);

	if (charged()) {
		_lastChargedTime = timestamp();
	}

	_local_obstacles.clear();
	resetMotionConstraints();
	_unkick();
}

void OurRobot::resetMotionConstraints() {
	_motionConstraints = MotionConstraints();
}

void OurRobot::stop() {
	resetMotionConstraints();

	*_cmdText << "stop()\n";
}

void OurRobot::move(const Geometry2d::Point &goal, float endSpeed)
{
	if (!visible)
		return;

	// sets flags for future movement
	if (verbose) cout << " in OurRobot::move(goal): adding a goal (" << goal.x << ", " << goal.y << ")" << std::endl;
	
	_motionConstraints.targetPos = goal;
	_motionConstraints.endSpeed = endSpeed;

	//	reset conflicting motion commands
	_motionConstraints.pivotTarget = boost::none;
	_motionConstraints.targetWorldVel = boost::none;

	// //	only invalidate path if move() is being called with a new goal or one wasn't set previously
	// if (!_motionConstraints.targetPos || !_motionConstraints.targetPos->nearPoint(goal, 0.02)) {
	// 	addText("Invalidated old path");
		
	// 	if (_motionConstraints.targetPos) {
	// 		addText(QString("Old goal: (%1, %2)").arg(goal.x, goal.y));
	// 		addText(QString("New goal: (%1, %2)").arg(_motionConstraints.targetPos->x, _motionConstraints.targetPos->y));
	// 	} else {
	// 		addText("Old goal was null");
	// 	}
		
	// 	_motionConstraints.targetPos = goal;
	// 	_pathInvalidated = true;
	// }

	*_cmdText << "move(" << goal.x << ", " << goal.y << ")\n";
	if (endSpeed != 0) {
		*_cmdText << "setEndSpeed(" << endSpeed << ")\n";
	}
}

void OurRobot::worldVelocity(const Geometry2d::Point& v)
{
	_motionConstraints.targetPos = boost::none;
	_motionConstraints.targetWorldVel = v;

	_path.reset();

	*_cmdText << "worldVel(" << v.x << ", " << v.y << ")\n";
}

void OurRobot::pivot(const Geometry2d::Point &pivotTarget) {
	_motionConstraints.pivotTarget = pivotTarget;

	//	reset other conflicting motion commands
	_motionConstraints.targetPos = boost::none;
	_motionConstraints.targetWorldVel = boost::none;
	_motionConstraints.faceTarget = boost::none;

	*_cmdText << "pivot(" << pivotTarget.x << ", " << pivotTarget.y << ")\n";
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

float OurRobot::kickTimer() const {
	return (charged()) ? 0.0 : (float)(timestamp() - _lastChargedTime) * TimestampToSecs;
}

void OurRobot::dribble(uint8_t speed)
{
	uint8_t scaled = *config->dribbler.multiplier * speed;
	radioTx.set_dribbler(scaled);

	*_cmdText << "dribble(" << (float)speed << ")\n";
}

void OurRobot::face(const Geometry2d::Point &pt)
{
	_motionConstraints.faceTarget = pt;

	//	reset conflicting motion commands
	_motionConstraints.pivotTarget = boost::none;

	*_cmdText << "face(" << pt.x << ", " << pt.y << ")\n";
}

void OurRobot::faceNone()
{
	_motionConstraints.faceTarget = boost::none;

	*_cmdText << "faceNone()\n";
}

void OurRobot::kick(uint8_t strength)
{
	_kick(strength);

	*_cmdText << "kick(" << (float)strength << ")\n";
}

void OurRobot::chip(uint8_t strength)
{
	_chip(strength);

	*_cmdText << "chip(" << (float)strength << ")\n";
}

void OurRobot::_kick(uint8_t strength) {
	uint8_t max = *config->kicker.maxKick;
	radioTx.set_kick(strength > max ? max : strength);
	radioTx.set_use_chipper(false);
}

void OurRobot::_chip(uint8_t strength) {
	uint8_t max = *config->kicker.maxChip;
	// TODO make sure we're not about to chip over the middle line.
	Segment robot_face_line = Segment(pos, pos + 10*Point::direction(angle * M_PI / 180.));
	Segment mid_field_line = Segment(Point(-Field_Width/2,Field_Length/2), Point(Field_Width/2,Field_Length/2));
	Point intersection;
	if(robot_face_line.intersects(mid_field_line, &intersection))
	{
		float dist = intersection.distTo(pos);
		int power = min(strength, chipPowerForDistance(dist));
		if(power == 0)
			_kick(strength);
		else
			strength = power;
	}
	radioTx.set_kick(strength > max ? max : strength);
	radioTx.set_use_chipper(true);
}

void OurRobot::_unkick() {
	_kick(0);
	_chip(0);
	radioTx.set_use_chipper(false);
	radioTx.set_kick_immediate(false);
}

void OurRobot::unkick()
{
	_unkick();

	*_cmdText << "unkick()\n";
}

void OurRobot::kickImmediately(bool im)
{
	radioTx.set_kick_immediate(im);
}


#pragma mark Robot Avoidance

void OurRobot::resetAvoidRobotRadii() {
	for (size_t i = 0; i < Num_Shells; ++i) {
		_self_avoid_mask[i] = (i != (size_t) shell()) ? *_selfAvoidRadius : -1.0;
		_opp_avoid_mask[i] = (i == state()->gameState.TheirInfo.goalie) ? *_oppGoalieAvoidRadius : *_oppAvoidRadius;
	}
}

void OurRobot::approachAllOpponents(bool enable) {
	BOOST_FOREACH(float &ar, _opp_avoid_mask)
		ar = (enable) ?  Opp_Avoid_Small : *_oppAvoidRadius;
}
void OurRobot::avoidAllOpponents(bool enable) {
	BOOST_FOREACH(float &ar, _opp_avoid_mask)
		ar = (enable) ?  -1.0 : *_oppAvoidRadius;
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
		_opp_avoid_mask[shell_id] = *_oppAvoidRadius;
	else
		_opp_avoid_mask[shell_id] = -1.0;
}

void OurRobot::approachOpponent(unsigned shell_id, bool enable_approach) {
	if (enable_approach)
		_opp_avoid_mask[shell_id] = Opp_Avoid_Small;
	else
		_opp_avoid_mask[shell_id] = *_oppAvoidRadius;
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

void OurRobot::shieldFromTeammates(float radius) {
	for (OurRobot *teammate : state()->self) {
		if (teammate) {
			teammate->avoidTeammateRadius(shell(), radius);
		}
	}
}



#pragma mark Ball Avoidance

void OurRobot::disableAvoidBall() {
	avoidBallRadius(-1);
}

void OurRobot::avoidBallRadius(float radius) {
	_avoidBallRadius = radius;

	*_cmdText << "avoidBall(" << radius << ")\n";
}

float OurRobot::avoidBallRadius() const {
	return _avoidBallRadius;
}

void OurRobot::resetAvoidBall() {
	avoidBallRadius(Ball_Avoid_Small);
}

std::shared_ptr<Geometry2d::Shape> OurRobot::createBallObstacle() const {
	// if game is stopped, large obstacle regardless of flags
	if (_state->gameState.state != GameState::Playing && !(_state->gameState.ourRestart || _state->gameState.theirPenalty()))
	{
		return std::shared_ptr<Geometry2d::Shape>(new Circle(_state->ball.pos, Field_CenterRadius));
	}

	// create an obstacle if necessary
	if (_avoidBallRadius > 0.0) {
		return std::shared_ptr<Geometry2d::Shape>(new Circle(_state->ball.pos, _avoidBallRadius));
	} else {
		return std::shared_ptr<Geometry2d::Shape>();
	}
}


#pragma mark Motion

void OurRobot::setPath(Planning::Path path) {
	_didSetPathThisIteration = true;

	_path = path;
	_pathInvalidated = false;
	_pathStartTime = timestamp();

	_path->endSpeed = _motionConstraints.endSpeed;
	_path->maxSpeed = _motionConstraints.maxSpeed;
	_path->maxAcceleration = _motionConstraints.maxAcceleration;

	//	start velocity is the speed we're going in the direction of the target start velocity
	Geometry2d::Point posOut, velOut;
	_path->startSpeed = 0;
	_path->evaluate(0.01, posOut, velOut);

	//	we don't let the start speed go below zero
	//	in reality we should allow for it, but motion control isn't there yet and negative values cause more error
	float startSpeed = vel.dot(velOut.normalized());
	_path->startSpeed = max<float>(startSpeed, 0);
}

int OurRobot::consecutivePathChangeCount() const {
	int count = 0;
	for (auto itr = _pathChangeHistory.begin(); itr != _pathChangeHistory.end(); itr++) {
		if (*itr) {
			count++;
		} else {
			break;
		}
	}

	return count > 0 ? count - 1 : 0;
}

void OurRobot::replanIfNeeded(const Geometry2d::CompositeShape& global_obstacles) {
	if (!_motionConstraints.targetPos) {
		_path = boost::none;
		return;
	}

	if (_state->gameState.state == GameState::Halt || !_motionConstraints.targetPos) {
		//	clear our history of path change times
		_pathChangeHistory.clear();

		return;
	}

	// create and visualize obstacles
	Geometry2d::CompositeShape full_obstacles(_local_obstacles);
	Geometry2d::CompositeShape
		self_obs = createRobotObstacles(_state->self, _self_avoid_mask),
		opp_obs = createRobotObstacles(_state->opp, _opp_avoid_mask);
	_state->drawCompositeShape(self_obs, Qt::gray, QString("self_obstacles_%1").arg(shell()));
	_state->drawCompositeShape(opp_obs, Qt::gray, QString("opp_obstacles_%1").arg(shell()));
	if (_state->ball.valid)
	{
		std::shared_ptr<Geometry2d::Shape> ball_obs = createBallObstacle();
		_state->drawShape(ball_obs, Qt::gray, QString("ball_obstacles_%1").arg(shell()));
		full_obstacles.add(ball_obs);
	}
	full_obstacles.add(self_obs);
	full_obstacles.add(opp_obs);
	full_obstacles.add(global_obstacles);

	// if no goal command robot to stop in place
	if (!_motionConstraints.targetPos) {
		if (verbose) cout << "in OurRobot::replanIfNeeded() for robot [" << shell() << "]: stopped" << std::endl;
		addText(QString("replan: no goal"));
		setPath(Planning::Path(pos));
		_state->drawPath(*_path);
		return;
	}


	Geometry2d::Point dest = *_motionConstraints.targetPos;

	// //	if this number of microseconds passes since our last path plan, we automatically replan
	// const uint64_t kPathExpirationInterval = 1.5 * SecsToTimestamp;
	// if ((timestamp() - _pathStartTime) > kPathExpirationInterval) {
	// 	_pathInvalidated = true;
	// }

	if (!_path) {
		_pathInvalidated = true;
	}


	Planning::Path newlyPlannedPath;
	_planner->run(pos, angle, vel, *_motionConstraints.targetPos, &full_obstacles, newlyPlannedPath);

	//	invalidate path if it hits obstacles
	//	TODO: it would be better to compare WHICH obstacles the old and new paths hit rather than just looking at IF they hit obstacles
	if (_path && _path->hit(full_obstacles) && !newlyPlannedPath.hit(full_obstacles)) {
		_pathInvalidated = true;
	}

	//  invalidate path if current position is more than 15cm from the planned point
	if (_path) {
		float maxDist = 0.30;
		Point targetPathPos;
		Point targetVel;
		float timeIntoPath = ((float)(timestamp() - _pathStartTime)) * TimestampToSecs;
		_path->evaluate(timeIntoPath, targetPathPos, targetVel);
		float pathError = (targetPathPos - pos).mag();
		if (pathError > maxDist) {
			_pathInvalidated = true;
		}
	}
	
	
	//	if the destination of the current path is greater than X m away from the target destination,
	//	we invalidate the path.  this situation could arise if during a previous planning, the target point
	//	was blocked by an obstacle
	if (_path && (_path->points.back() - dest).mag() > 0.025) {
		_pathInvalidated = true;
	}


	//	try a straight path EVERY time
	if (_path && _path->points.size() > 2) {
		//	try a straight line path first
		Geometry2d::Segment straight_seg(pos, *_motionConstraints.targetPos);
		if (!full_obstacles.hit(straight_seg)) {
			addText(QString("planner: pre-emptive straight_line"));
			Planning::Path straightLine(pos, *_motionConstraints.targetPos);
			setPath(straightLine);
			_pathInvalidated = false;
		}
	}

	


	// check if goal is close to previous goal to reuse path
	if (!_pathInvalidated) {
		addText("Reusing path");
		// for (auto itr : _path->points) {
		// 	cout << "\t(" << itr.x << ", " << itr.y << ")" << endl;
		// }
	} else {
		// use the newly generated path
		if (verbose) cout << "in OurRobot::replanIfNeeded() for robot [" << shell() << "]: using new RRT path" << std::endl;
		
		//	try a straight line path first
		Geometry2d::Segment straight_seg(pos, *_motionConstraints.targetPos);
		if (!full_obstacles.hit(straight_seg)) {
			addText(QString("planner: straight_line"));
			Planning::Path straightLine(pos, *_motionConstraints.targetPos);
			setPath(straightLine);
		} else {
			//	rrt-planned path
			setPath(newlyPlannedPath);
		}
	}


	_path->maxSpeed = _motionConstraints.maxSpeed;
	_path->endSpeed = _motionConstraints.endSpeed;
	_path->maxAcceleration = _motionConstraints.maxAcceleration;

	_state->drawPath(*_path, Qt::magenta);

	_pathChangeHistory.push_back(_didSetPathThisIteration);

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
	if (_radioRx.has_quaternion() && rxIsFresh(0.05 * SecsToTimestamp))
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

void OurRobot::radioRxUpdated() {
	if ( _radioRx.kicker_status() < _lastKickerStatus ) {
		_lastKickTime = timestamp();
	}
	_lastKickerStatus = _radioRx.kicker_status();
}

double OurRobot::distanceToChipLanding(int chipPower) {
	return max(0., min(190., *(config->chipper.calibrationSlope) * chipPower + *(config->chipper.calibrationOffset)));
}

uint8_t OurRobot::chipPowerForDistance(double distance) {
	double b = *(config->chipper.calibrationOffset) / 2.;
	if(distance < b)
		return 0;
	if(distance > distanceToChipLanding(255))
		return 255;
	return 0.5 * distance + b;
}