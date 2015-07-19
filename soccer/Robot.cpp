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
#include <QString>
#include <cmath>
#include <utility>
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
ConfigDouble *OurRobot::_goalChangeThreshold;
ConfigDouble *OurRobot::_replanTimeout;

void OurRobot::createConfiguration(Configuration *cfg) {
	_selfAvoidRadius = new ConfigDouble(cfg, "PathPlanner/selfAvoidRadius", Robot_Radius);
	_oppAvoidRadius = new ConfigDouble(cfg, "PathPlanner/oppAvoidRadius", Robot_Radius - 0.01);
	_oppGoalieAvoidRadius = new ConfigDouble(cfg, "PathPlanner/oppGoalieAvoidRadius", Robot_Radius + 0.05);

	_replanTimeout = new ConfigDouble(cfg, "PathPlanner/replanTimeout", 5);
	_goalChangeThreshold = new ConfigDouble(cfg, "PathPlanner/goalChangeThreshold", 0.025);
}

OurRobot::OurRobot(int shell, SystemState *state):
	Robot(shell, true),
	_path(),
	_state(state),
	_pathChangeHistory(PathChangeHistoryBufferSize)
{
	_cmdText = new std::stringstream();

	resetAvoidBall();
	_lastChargedTime = 0;
	_lastKickerStatus = 0;
	_lastKickTime = 0;

	_motionControl = new MotionControl(this);

	_planner = make_shared<Planning::RRTPlanner>(250);

	resetAvoidRobotRadii();

	_clearCmdText();
}

/**
 * ourrobot deconstructor, deletes motion control and planner
 */
OurRobot::~OurRobot()
{
	if (_motionControl) delete _motionControl;
	delete _cmdText;
}

void OurRobot::addStatusText()
{
	const QColor statusColor(255, 32, 32);

	if (!rxIsFresh())
	{
		addText("No RX", statusColor, "Status");

		// No more status is available
		return;
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
	for (float &a :  _opp_avoid_mask)
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

void OurRobot::move(const Geometry2d::Point &goal, Geometry2d::Point endVelocity)
{
	if (!visible)
		return;

	// sets flags for future movement
	if (verbose) cout << " in OurRobot::move(goal): adding a goal (" << goal.x << ", " << goal.y << ")" << std::endl;

	_motionCommand.setPathTarget(Planning::MotionInstant(goal, endVelocity));

	//	reset conflicting motion commands
	_motionConstraints.pivotTarget = boost::none;

	*_cmdText << "move(" << goal.x << ", " << goal.y << ")\n";
	*_cmdText << "endVelocity(" << endVelocity.x << ")\n";
}

void OurRobot::worldVelocity(const Geometry2d::Point& v)
{
	_motionCommand.setWorldVel(v);
	setPath(NULL);
	*_cmdText << "worldVel(" << v.x << ", " << v.y << ")\n";
}



void OurRobot::angleVelocity(float targetAngleVel) {
	_motionConstraints.targetAngleVel = fixAngleRadians(targetAngleVel);

	//	reset other conflicting motion commands
	_motionConstraints.faceTarget = boost::none;
	_motionConstraints.pivotTarget = boost::none;

	*_cmdText << "angleVelocity(" << targetAngleVel << ")\n";
}



void OurRobot::pivot(const Geometry2d::Point &pivotTarget) {
	_motionConstraints.pivotTarget = pivotTarget;

	//	reset other conflicting motion commands
	_motionCommand.setWorldVel(Geometry2d::Point());
	_motionConstraints.faceTarget = boost::none;
	setPath(NULL);

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

void OurRobot::kick(float strength)
{
	double maxKick = *config->kicker.maxKick;
	_kick(roundf(strength*((float)maxKick)));

	*_cmdText << "kick(" << strength*100 << "%)\n";
}

void OurRobot::kickLevel(uint8_t strength)
{
	_kick(strength);

	*_cmdText << "kick(" << (float)strength << ")\n";
}

void OurRobot::chip(float strength)
{
	double maxChip = *config->kicker.maxChip;
	_chip(roundf(strength*((float)maxChip)));
	*_cmdText << "chip(" << strength*100 << "%)\n";
}

void OurRobot::chipLevel(uint8_t strength)
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
	for (float &ar :  _opp_avoid_mask)
		ar = (enable) ?  Opp_Avoid_Small : *_oppAvoidRadius;
}
void OurRobot::avoidAllOpponents(bool enable) {
	for (float &ar :  _opp_avoid_mask)
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
	for (float &ar :  _opp_avoid_mask)
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
		return std::shared_ptr<Geometry2d::Shape>(new Circle(_state->ball.pos, Field_Dimensions::Current_Dimensions.CenterRadius()));
	}

	// create an obstacle if necessary
	if (_avoidBallRadius > 0.0) {
		return std::shared_ptr<Geometry2d::Shape>(new Circle(_state->ball.pos, _avoidBallRadius));
	} else {
		return std::shared_ptr<Geometry2d::Shape>();
	}
}


#pragma mark Motion

void OurRobot::setPath(unique_ptr<Planning::Path> path) {
	_didSetPathThisIteration = true;

	_path = std::move(path);
	_pathInvalidated = false;
	_pathStartTime = timestamp();
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
	// if no goal command robot to stop in place
	if (_state->gameState.state == GameState::Halt) {
		//	clear our history of path change times
		_pathChangeHistory.clear();
		setPath(nullptr);
		return;
	}

	if (!_motionCommand.usePathPlanning()) {
		setPath(nullptr);
		return;
	}

	// create and visualize obstacles
	Geometry2d::CompositeShape full_obstacles(_local_obstacles);
	//Add's our robots as obstacles only if they're within a certain distance from our robot.
	//This distance increases with velocity.
	Geometry2d::CompositeShape
		self_obs = createRobotObstacles(_state->self, _self_avoid_mask, this->pos, 0.6 + this->vel.mag()),
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

	Planning::MotionInstant commandDestination = _motionCommand.getPlanningTarget();

	// //	if this number of microseconds passes since our last path plan, we automatically replan
	const Time kPathExpirationInterval = *_replanTimeout * SecsToTimestamp;
	if ((timestamp() - _pathStartTime) > kPathExpirationInterval) {
		_pathInvalidated = true;
	}

	if (!_path) {
		_pathInvalidated = true;
	} else {

		Planning::MotionInstant target;
		float timeIntoPath = ((float)(timestamp() - _pathStartTime)) * TimestampToSecs + 1.0f/60.0f;
		_path->evaluate(timeIntoPath, target);

		float pathError = (target.pos - pos).mag();
		float replanThreshold = *_motionConstraints._replan_threshold;
		state()->drawCircle(target.pos, replanThreshold, Qt::green, "MotionControl");
		addText(QString("velocity: %1 %2").arg(this->vel.x).arg(this->vel.y));

		if (*_motionConstraints._replan_threshold!=0 && pathError > replanThreshold) {
			_pathInvalidated = true;
			addText("pathError" , Qt::red, "Motion");
			//addText(pathError);
		}


		if (std::isnan(target.pos.x) || std::isnan(target.pos.y)) {
			_pathInvalidated = true;
			addText("Evaulate Returned an invalid result" , Qt::red, "Motion");
			//addText(pathError);
		}


		float hitTime=0;
		if (_path->hit(full_obstacles, hitTime, timeIntoPath)) {
			_pathInvalidated = true;
			addText("Hit Obstacle", Qt::red, "Motion");
		}

		//  invalidate path if current position is more than 15cm from the planned point



		//	if the destination of the current path is greater than X m away from the target destination,
		//	we invalidate the path.  this situation could arise if the path destination changed
		if ((_path->destination()->pos - commandDestination.pos).mag() > *_goalChangeThreshold ||
				(_path->destination()->vel - commandDestination.vel).mag() > *_goalChangeThreshold) {
			_pathInvalidated = true;
		}
	}


	// check if goal is close to previous goal to reuse path
	if (!_pathInvalidated) {
		addText("Reusing path", Qt::white, "Planning");
	} else {
		double leadTime = *(_motionConstraints._replan_lead_time) * 1000000;
		RobotPose predictedPose;

		filter()->predict(timestamp() + leadTime, &predictedPose);
		std::unique_ptr<Planning::Path> path = nullptr;
		int count = 0;
		while (!path) {
			path = _planner->run(Planning::MotionInstant(pos, vel), _motionCommand.getPlanningTarget(), _motionConstraints, &full_obstacles);
			count++;
			//TODO fix this
			if (count >=50) {
				path = unique_ptr<Planning::Path>(new Planning::InterpolatedPath());
			}
		}

		addText("Replanning", Qt::red, "Planning");

		// use the newly generated path
		if (verbose) cout << "in OurRobot::replanIfNeeded() for robot [" << shell() << "]: using new RRT path" << std::endl;
		setPath(std::move(path));
	}

	if (_path) {
		_path->draw(_state, Qt::magenta);
	}
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

bool OurRobot::rxIsFresh(Time age) const
{
	return (timestamp() - _radioRx.timestamp()) < age;
}

Time OurRobot::lastKickTime() const {
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
