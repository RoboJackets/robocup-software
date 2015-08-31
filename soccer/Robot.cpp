#include <Robot.hpp>
#include <LogUtils.hpp>
#include <modeling/RobotFilter.hpp>
#include <motion/MotionControl.hpp>
#include <planning/TrapezoidalPath.hpp>
#include <protobuf/LogFrame.pb.h>
#include <RobotConfig.hpp>
#include <SystemState.hpp>
#include <Utils.hpp>

#include <cmath>
#include <execinfo.h>
#include <iostream>
#include <QString>
#include <stdexcept>
#include <stdio.h>
#include <utility>

using namespace std;
using namespace Geometry2d;
using Planning::MotionInstant;

/** thresholds for avoidance of opponents - either a normal (large) or an
 * approach (small)*/
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
Robot::Robot(unsigned int shell, bool self) {
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
Robot::~Robot() {
    delete _filter;
    _filter = nullptr;
}

#pragma mark OurRobot

REGISTER_CONFIGURABLE(OurRobot)

ConfigDouble* OurRobot::_selfAvoidRadius;
ConfigDouble* OurRobot::_oppAvoidRadius;
ConfigDouble* OurRobot::_oppGoalieAvoidRadius;
ConfigDouble* OurRobot::_goalChangeThreshold;
ConfigDouble* OurRobot::_replanTimeout;

void OurRobot::createConfiguration(Configuration* cfg) {
    _selfAvoidRadius =
        new ConfigDouble(cfg, "PathPlanner/selfAvoidRadius", Robot_Radius);
    _oppAvoidRadius = new ConfigDouble(cfg, "PathPlanner/oppAvoidRadius",
                                       Robot_Radius - 0.01);
    _oppGoalieAvoidRadius = new ConfigDouble(
        cfg, "PathPlanner/oppGoalieAvoidRadius", Robot_Radius + 0.05);

    _replanTimeout = new ConfigDouble(cfg, "PathPlanner/replanTimeout", 5);
    _goalChangeThreshold =
        new ConfigDouble(cfg, "PathPlanner/goalChangeThreshold", 0.025);
}

OurRobot::OurRobot(int shell, SystemState* state)
    : Robot(shell, true), _path(), _state(state) {
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
OurRobot::~OurRobot() {
    if (_motionControl) delete _motionControl;
    delete _cmdText;
}

void OurRobot::addStatusText() {
    const QColor statusColor(255, 32, 32);

    if (!rxIsFresh()) {
        addText("No RX", statusColor, "Status");

        // No more status is available
        return;
    }
}

void OurRobot::addText(const QString& text, const QColor& qc,
                       const QString& layerPrefix) {
    Packet::DebugText* dbg = new Packet::DebugText;
    QString layer = layerPrefix + QString::number(shell());
    dbg->set_layer(_state->findDebugLayer(layer));
    dbg->set_text(text.toStdString());
    dbg->set_color(color(qc));
    robotText.push_back(dbg);
}

bool OurRobot::avoidOpponents() const {
    // checks for avoiding all opponents
    for (size_t i = 0; i < Num_Shells; ++i)
        if (_state->opp[i] && _state->opp[i]->visible &&
            _opp_avoid_mask[i] < 0.1)
            return false;
    return true;
}

void OurRobot::avoidOpponents(bool enable) {
    for (float& a : _opp_avoid_mask)
        if (enable)
            a = Robot_Radius - 0.03;
        else
            a = -1.0;
}

std::string OurRobot::getCmdText() const { return _cmdText->str(); }

void OurRobot::_clearCmdText() {
    _cmdText->str("");
    _cmdText->clear();
}

void OurRobot::resetForNextIteration() {
    if (verbose && visible)
        cout << "in OurRobot::resetForNextIteration()" << std::endl;
    robotText.clear();

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

    isPenaltyKicker = false;
}

void OurRobot::resetMotionConstraints() {
    _motionConstraints = MotionConstraints();
    _motionCommand = Planning::MotionCommand();
}

void OurRobot::stop() {
    resetMotionConstraints();

    *_cmdText << "stop()\n";
}

void OurRobot::move(Geometry2d::Point goal, Geometry2d::Point endVelocity) {
    if (!visible) return;

    // sets flags for future movement
    if (verbose)
        cout << " in OurRobot::move(goal): adding a goal (" << goal.x << ", "
             << goal.y << ")" << std::endl;

    _motionCommand.setPathTarget(MotionInstant(goal, endVelocity));

    // reset conflicting motion commands
    _motionConstraints.pivotTarget = boost::none;

    *_cmdText << "move(" << goal.x << ", " << goal.y << ")" << endl;
    *_cmdText << "endVelocity(" << endVelocity.x << ", " << endVelocity.y << ")"
              << endl;
}

void OurRobot::moveDirect(Geometry2d::Point goal, float endSpeed) {
    if (!visible) return;

    // sets flags for future movement
    if (verbose)
        cout << " in OurRobot::moveDirect(goal): adding a goal (" << goal.x
             << ", " << goal.y << ")" << endl;

    _motionCommand.setDirectTarget(goal, endSpeed);

    // reset conflicting motion commands
    _motionConstraints.pivotTarget = boost::none;

    *_cmdText << "moveDirect(" << goal.x << ", " << goal.y << ")" << endl;
    *_cmdText << "endSpeed(" << endSpeed << ")" << endl;
}

void OurRobot::worldVelocity(Geometry2d::Point v) {
    _motionCommand.setWorldVel(v);
    setPath(nullptr);
    *_cmdText << "worldVel(" << v.x << ", " << v.y << ")" << endl;
}

void OurRobot::angleVelocity(float targetAngleVel) {
    _motionConstraints.targetAngleVel = fixAngleRadians(targetAngleVel);

    // reset other conflicting motion commands
    _motionConstraints.faceTarget = boost::none;
    _motionConstraints.pivotTarget = boost::none;

    *_cmdText << "angleVelocity(" << targetAngleVel << ")" << endl;
}

void OurRobot::pivot(Geometry2d::Point pivotTarget) {
    _motionConstraints.pivotTarget = pivotTarget;

    // reset other conflicting motion commands
    _motionCommand.setWorldVel(Geometry2d::Point());
    _motionConstraints.faceTarget = boost::none;
    setPath(nullptr);

    *_cmdText << "pivot(" << pivotTarget.x << ", " << pivotTarget.y << ")"
              << endl;
}

Geometry2d::Point OurRobot::pointInRobotSpace(Geometry2d::Point pt) const {
    Point p = pt;
    p.rotate(pos, -angle);
    return p;
}

const Geometry2d::Segment OurRobot::kickerBar() const {
    TransformMatrix pose(pos, angle);
    const float mouthHalf = Robot_MouthWidth / 2.0f;
    float x = sin(acos(mouthHalf / Robot_Radius)) * Robot_Radius;
    Point L(x, Robot_MouthWidth / 2.0f);
    Point R(x, -Robot_MouthWidth / 2.0f);
    return Segment(pose * L, pose * R);
}

bool OurRobot::behindBall(Geometry2d::Point ballPos) const {
    Point ballTransformed = pointInRobotSpace(ballPos);
    return ballTransformed.x < -Robot_Radius;
}

float OurRobot::kickTimer() const {
    return (charged()) ? 0.0 : (float)(timestamp() - _lastChargedTime) *
                                   TimestampToSecs;
}

void OurRobot::dribble(uint8_t speed) {
    uint8_t scaled = *config->dribbler.multiplier * speed;
    radioTx.set_dribbler(scaled);

    *_cmdText << "dribble(" << (float)speed << ")" << endl;
}

void OurRobot::face(Geometry2d::Point pt) {
    _motionConstraints.faceTarget = pt;

    // reset conflicting motion commands
    _motionConstraints.pivotTarget = boost::none;

    *_cmdText << "face(" << pt.x << ", " << pt.y << ")" << endl;
}

void OurRobot::faceNone() {
    _motionConstraints.faceTarget = boost::none;

    *_cmdText << "faceNone()" << endl;
}

void OurRobot::kick(float strength) {
    double maxKick = *config->kicker.maxKick;
    _kick(roundf(strength * ((float)maxKick)));

    *_cmdText << "kick(" << strength * 100 << "%)" << endl;
}

void OurRobot::kickLevel(uint8_t strength) {
    _kick(strength);

    *_cmdText << "kick(" << (float)strength << ")" << endl;
}

void OurRobot::chip(float strength) {
    double maxChip = *config->kicker.maxChip;
    _chip(roundf(strength * ((float)maxChip)));
    *_cmdText << "chip(" << strength * 100 << "%)" << endl;
}

void OurRobot::chipLevel(uint8_t strength) {
    _chip(strength);

    *_cmdText << "chip(" << (float)strength << ")" << endl;
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

void OurRobot::unkick() {
    _unkick();

    *_cmdText << "unkick()" << endl;
}

void OurRobot::kickImmediately(bool im) { radioTx.set_kick_immediate(im); }

#pragma mark Robot Avoidance

void OurRobot::resetAvoidRobotRadii() {
    for (size_t i = 0; i < Num_Shells; ++i) {
        _self_avoid_mask[i] = (i != (size_t)shell()) ? *_selfAvoidRadius : -1.0;
        _opp_avoid_mask[i] = (i == state()->gameState.TheirInfo.goalie)
                                 ? *_oppGoalieAvoidRadius
                                 : *_oppAvoidRadius;
    }
}

void OurRobot::approachAllOpponents(bool enable) {
    for (float& ar : _opp_avoid_mask)
        ar = (enable) ? Opp_Avoid_Small : *_oppAvoidRadius;
}
void OurRobot::avoidAllOpponents(bool enable) {
    for (float& ar : _opp_avoid_mask) ar = (enable) ? -1.0 : *_oppAvoidRadius;
}

bool OurRobot::avoidOpponent(unsigned shell_id) const {
    return _opp_avoid_mask[shell_id] > 0.0;
}

bool OurRobot::approachOpponent(unsigned shell_id) const {
    return avoidOpponent(shell_id) &&
           _opp_avoid_mask[shell_id] < Robot_Radius - 0.01;
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
    for (float& ar : _opp_avoid_mask) ar = radius;
}

void OurRobot::avoidAllTeammates(bool enable) {
    for (size_t i = 0; i < Num_Shells; ++i) avoidTeammate(i, enable);
}
void OurRobot::avoidTeammate(unsigned shell_id, bool enable) {
    if (shell_id != shell())
        _self_avoid_mask[shell_id] = (enable) ? Robot_Radius : -1.0;
}

void OurRobot::avoidTeammateRadius(unsigned shell_id, float radius) {
    if (shell_id != shell()) _self_avoid_mask[shell_id] = radius;
}

bool OurRobot::avoidTeammate(unsigned shell_id) const {
    return _self_avoid_mask[shell_id] < Robot_Radius;
}

float OurRobot::avoidTeammateRadius(unsigned shell_id) const {
    return _self_avoid_mask[shell_id];
}

void OurRobot::shieldFromTeammates(float radius) {
    for (OurRobot* teammate : state()->self) {
        if (teammate) {
            teammate->avoidTeammateRadius(shell(), radius);
        }
    }
}

#pragma mark Ball Avoidance

void OurRobot::disableAvoidBall() { avoidBallRadius(-1); }

void OurRobot::avoidBallRadius(float radius) {
    _avoidBallRadius = radius;

    *_cmdText << "avoidBall(" << radius << ")" << endl;
}

float OurRobot::avoidBallRadius() const { return _avoidBallRadius; }

void OurRobot::resetAvoidBall() { avoidBallRadius(Ball_Avoid_Small); }

std::shared_ptr<Geometry2d::Shape> OurRobot::createBallObstacle() const {
    // if game is stopped, large obstacle regardless of flags
    if (_state->gameState.state != GameState::Playing &&
        !(_state->gameState.ourRestart || _state->gameState.theirPenalty())) {
        return std::shared_ptr<Geometry2d::Shape>(
            new Circle(_state->ball.pos,
                       Field_Dimensions::Current_Dimensions.CenterRadius()));
    }

    // create an obstacle if necessary
    if (_avoidBallRadius > 0.0) {
        return std::shared_ptr<Geometry2d::Shape>(
            new Circle(_state->ball.pos, _avoidBallRadius));
    } else {
        return std::shared_ptr<Geometry2d::Shape>();
    }
}

#pragma mark Motion

void OurRobot::setPath(unique_ptr<Planning::Path> path) {
    _path = std::move(path);
    _pathInvalidated = false;
}

void OurRobot::replanIfNeeded(const Geometry2d::ShapeSet& globalObstacles) {
    Planning::MotionCommand::CommandType lastCommandType = _lastCommandType;
    _lastCommandType = _motionCommand.getCommandType();

    // if no goal, command robot to stop in place
    if (_state->gameState.state == GameState::Halt) {
        setPath(nullptr);
        return;
    }

    if (_motionCommand.getCommandType() == Planning::MotionCommand::WorldVel) {
        setPath(nullptr);
        return;
    }

    // create and visualize obstacles
    Geometry2d::ShapeSet fullObstacles(_local_obstacles);
    // Adds our robots as obstacles only if they're within a certain distance
    // from this robot. This distance increases with velocity.
    const Geometry2d::ShapeSet selfObs = createRobotObstacles(
        _state->self, _self_avoid_mask, this->pos, 0.6 + this->vel.mag());
    const Geometry2d::ShapeSet oppObs =
        createRobotObstacles(_state->opp, _opp_avoid_mask);

    if (_state->ball.valid) {
        std::shared_ptr<Geometry2d::Shape> ball_obs = createBallObstacle();
        _state->drawShape(ball_obs, Qt::gray,
                          QString("ball_obstacles_%1").arg(shell()));
        fullObstacles.add(ball_obs);
    }
    fullObstacles.add(selfObs);
    fullObstacles.add(oppObs);
    fullObstacles.add(globalObstacles);

    _state->drawShapeSet(selfObs, Qt::gray,
                         QString("self_obstacles_%1").arg(shell()));
    _state->drawShapeSet(oppObs, Qt::gray,
                         QString("opp_obstacles_%1").arg(shell()));
    if (_path && lastCommandType == _motionCommand.getCommandType()) {
        if (_motionCommand.getCommandType() ==
            Planning::MotionCommand::PathTarget) {
            MotionInstant commandDestination =
                _motionCommand.getPlanningTarget();

            // if this number of microseconds passes since our last path plan,
            // we automatically replan
            const Time kPathExpirationInterval =
                *_replanTimeout * SecsToTimestamp;
            if ((timestamp() - _path->startTime()) > kPathExpirationInterval) {
                _pathInvalidated = true;
            }

            MotionInstant target;
            float timeIntoPath =
                ((float)(timestamp() - _path->startTime())) * TimestampToSecs +
                1.0f / 60.0f;

            boost::optional<MotionInstant> optTarget =
                _path->evaluate(timeIntoPath);
            if (optTarget) {
                target = *optTarget;
            } else {
                // We went off the end of the path, so use the end for
                // calculations.
                target = *_path->destination();
            }

            float pathError = (target.pos - pos).mag();
            float replanThreshold = *_motionConstraints._replan_threshold;
            state()->drawCircle(target.pos, replanThreshold, Qt::green,
                                "MotionControl");
            addText(
                QString("velocity: %1 %2").arg(this->vel.x).arg(this->vel.y));

            //  invalidate path if current position is more than the
            //  replanThreshold
            if (*_motionConstraints._replan_threshold != 0 &&
                pathError > replanThreshold) {
                _pathInvalidated = true;
                addText("pathError", Qt::red, "Motion");
            }

            if (std::isnan(target.pos.x) || std::isnan(target.pos.y)) {
                _pathInvalidated = true;
                addText("Evaulate Returned an invalid result", Qt::red,
                        "Motion");
            }

            float hitTime = 0;
            if (_path->hit(fullObstacles, hitTime, timeIntoPath)) {
                _pathInvalidated = true;
                addText("Hit Obstacle", Qt::red, "Motion");
            }

            // if the destination of the current path is greater than X m away
            // from the target destination, we invalidate the path. This
            // situation could arise if the path destination changed.
            if (!_path->destination() ||
                (_path->destination()->pos - commandDestination.pos).mag() >
                    *_goalChangeThreshold ||
                (_path->destination()->vel - commandDestination.vel).mag() >
                    *_goalChangeThreshold) {
                _pathInvalidated = true;
            }
        } else if (_motionCommand.getCommandType() ==
                   Planning::MotionCommand::DirectTarget) {
            Geometry2d::Point endTarget;
            float endSpeed = _motionCommand.getDirectTarget(endTarget);
            if (!_path->destination() ||
                (_path->destination()->pos - endTarget).mag() >
                    *_goalChangeThreshold ||
                (_path->destination()->vel.mag() - endSpeed) >
                    *_goalChangeThreshold) {
                _pathInvalidated = true;
            }
        } else {
            _pathInvalidated = true;
        }
    } else {
        _pathInvalidated = true;
    }

    if (!_pathInvalidated) {
        addText("Reusing path", Qt::white, "Planning");
    } else {
        Time leadTime =
            *(_motionConstraints._replan_lead_time) * SecsToTimestamp;
        RobotPose predictedPose;

        filter()->predict(timestamp() + leadTime, &predictedPose);
        std::unique_ptr<Planning::Path> path = nullptr;
        int planning_attempts = 0;
        while (!path) {
            Geometry2d::Point endTarget;
            float endSpeed = _motionCommand.getDirectTarget(endTarget);
            switch (_motionCommand.getCommandType()) {
                case Planning::MotionCommand::PathTarget:
                    path = _planner->run(MotionInstant(pos, vel),
                                         _motionCommand.getPlanningTarget(),
                                         _motionConstraints, &fullObstacles);
                    break;
                case Planning::MotionCommand::DirectTarget:
                    path = unique_ptr<Planning::Path>(
                        new Planning::TrapezoidalPath(
                            this->pos, this->vel.mag(), endTarget, endSpeed,
                            _motionConstraints));
                    path->setStartTime(timestamp());
                    break;
                default:
                    path = nullptr;
            }
            planning_attempts++;

            // TODO: fix this
            // Due to a bug in the path planner, sometimes planning is
            // successful, other times it fails due to issues with NaN.
            // Planning happens in a loop here so we can retry for a limited
            // number of times.
            if (planning_attempts >= 50) {
                path = nullptr;
                addText("PathPlanning Failed", Qt::red, "Planning");
                break;
            }
        }

        addText("Replanning", Qt::red, "Planning");
        // use the newly generated path
        if (verbose)
            cout << "in OurRobot::replanIfNeeded() for robot [" << shell()
                 << "]: using new RRT path" << std::endl;
        setPath(std::move(path));
    }

    if (_path) {
        _path->draw(_state, Qt::magenta, "Planning");
    }
}

bool OurRobot::charged() const {
    return _radioRx.has_kicker_status() && (_radioRx.kicker_status() & 0x01) &&
           rxIsFresh();
}

bool OurRobot::hasBall() const {
    return _radioRx.has_ball_sense_status() &&
           _radioRx.ball_sense_status() == Packet::HasBall && rxIsFresh();
}

bool OurRobot::ballSenseWorks() const {
    return rxIsFresh() && _radioRx.has_ball_sense_status() &&
           (_radioRx.ball_sense_status() == Packet::NoBall ||
            _radioRx.ball_sense_status() == Packet::HasBall);
}

bool OurRobot::kickerWorks() const {
    return _radioRx.has_kicker_status() && !(_radioRx.kicker_status() & 0x80) &&
           rxIsFresh();
}

bool OurRobot::chipper_available() const {
    return hardwareVersion() == Packet::RJ2011 && kickerWorks() &&
           *status->chipper_enabled;
}

bool OurRobot::kicker_available() const {
    return kickerWorks() && *status->kicker_enabled;
}

bool OurRobot::dribbler_available() const {
    return *status->dribbler_enabled && _radioRx.motor_status_size() == 5 &&
           _radioRx.motor_status(4) == Packet::Good;
}

bool OurRobot::driving_available(bool require_all) const {
    if (_radioRx.motor_status_size() != 5) return false;
    int c = 0;
    for (int i = 0; i < 4; ++i) {
        if (_radioRx.motor_status(i) == Packet::Good) {
            ++c;
        }
    }
    return (require_all) ? c == 4 : c == 3;
}

float OurRobot::kickerVoltage() const {
    if (_radioRx.has_kicker_voltage() && rxIsFresh()) {
        return _radioRx.kicker_voltage();
    } else {
        return 0;
    }
}

Packet::HardwareVersion OurRobot::hardwareVersion() const {
    if (rxIsFresh()) {
        return _radioRx.hardware_version();
    } else {
        return Packet::Unknown;
    }
}

boost::optional<Eigen::Quaternionf> OurRobot::quaternion() const {
    if (_radioRx.has_quaternion() && rxIsFresh(0.05 * SecsToTimestamp)) {
        return Eigen::Quaternionf(_radioRx.quaternion().q0() / 16384.0,
                                  _radioRx.quaternion().q1() / 16384.0,
                                  _radioRx.quaternion().q2() / 16384.0,
                                  _radioRx.quaternion().q3() / 16384.0);
    } else {
        return boost::none;
    }
}

bool OurRobot::rxIsFresh(Time age) const {
    return (timestamp() - _radioRx.timestamp()) < age;
}

Time OurRobot::lastKickTime() const { return _lastKickTime; }

void OurRobot::radioRxUpdated() {
    if (_radioRx.kicker_status() < _lastKickerStatus) {
        _lastKickTime = timestamp();
    }
    _lastKickerStatus = _radioRx.kicker_status();
}

double OurRobot::distanceToChipLanding(int chipPower) {
    return max(0., min(190., *(config->chipper.calibrationSlope) * chipPower +
                                 *(config->chipper.calibrationOffset)));
}

uint8_t OurRobot::chipPowerForDistance(double distance) {
    double b = *(config->chipper.calibrationOffset) / 2.;
    if (distance < b) return 0;
    if (distance > distanceToChipLanding(255)) return 255;
    return 0.5 * distance + b;
}
