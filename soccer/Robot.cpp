#include <Robot.hpp>
#include <LogUtils.hpp>
#include <modeling/RobotFilter.hpp>
#include <motion/MotionControl.hpp>
#include <planning/RRTPlanner.hpp>
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

Robot::Robot(unsigned int shell, bool self)
    : RobotPose(), _shell(shell), _self(self), _filter(new RobotFilter()) {}

Robot::~Robot() {
    delete _filter;
    _filter = nullptr;
}

#pragma mark OurRobot

REGISTER_CONFIGURABLE(OurRobot)

ConfigDouble* OurRobot::_selfAvoidRadius;
ConfigDouble* OurRobot::_oppAvoidRadius;
ConfigDouble* OurRobot::_oppGoalieAvoidRadius;

void OurRobot::createConfiguration(Configuration* cfg) {
    _selfAvoidRadius =
        new ConfigDouble(cfg, "PathPlanner/selfAvoidRadius", Robot_Radius);
    _oppAvoidRadius = new ConfigDouble(cfg, "PathPlanner/oppAvoidRadius",
                                       Robot_Radius - 0.01);
    _oppGoalieAvoidRadius = new ConfigDouble(
        cfg, "PathPlanner/oppGoalieAvoidRadius", Robot_Radius + 0.05);
}

OurRobot::OurRobot(int shell, SystemState* state)
    : Robot(shell, true), _state(state) {
    _cmdText = new std::stringstream();
    Packet::Control* ctl = new Packet::Control();
    robotPacket.set_allocated_control(ctl);
    control = ctl;

    //_lastChargedTime = 0;
    _lastKickerStatus = 0;
    //_lastKickTime = 0;
    _lastBallSense = RJ::Time();

    _motionControl = new MotionControl(this);

    resetAvoidRobotRadii();

    _clearCmdText();
}

OurRobot::~OurRobot() {
    if (_motionControl) delete _motionControl;
    delete _cmdText;
}

void OurRobot::addStatusText() {
    const QColor statusColor(255, 32, 32);

    if (!rxIsFresh()) {
        addText("No RX", statusColor, "Status");
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
    for (size_t i = 0; i < Num_Shells; ++i) {
        if (_state->opp[i] && _state->opp[i]->visible &&
            _opp_avoid_mask[i] < 0.1)
            return false;
    }
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

    control->Clear();
    control->set_dvelocity(0);
    robotPacket.set_uid(shell());

    if (charged()) {
        _lastChargedTime = RJ::now();
    }

    _local_obstacles.clear();

    resetMotionConstraints();
    _unkick();
    control->set_song(Packet::Control::STOP);

    isPenaltyKicker = false;
    isBallPlacer = false;
}

void OurRobot::resetMotionConstraints() {
    _robotConstraints = RobotConstraints();
    _motionCommand = std::make_unique<Planning::EmptyCommand>();
    _rotationCommand = std::make_unique<Planning::EmptyAngleCommand>();
    _planningPriority = 0;
}

void OurRobot::stop() {
    resetMotionConstraints();

    *_cmdText << "stop()\n";
}

void OurRobot::moveDirect(Geometry2d::Point goal, float endSpeed) {
    if (!visible) return;

    // sets flags for future movement
    if (verbose)
        cout << " in OurRobot::moveDirect(goal): adding a goal (" << goal.x()
             << ", " << goal.y() << ")" << endl;

    _motionCommand = std::make_unique<Planning::DirectPathTargetCommand>(
        MotionInstant(goal, (goal - pos).normalized() * endSpeed));

    *_cmdText << "moveDirect(" << goal << ")" << endl;
    *_cmdText << "endSpeed(" << endSpeed << ")" << endl;
}

void OurRobot::moveTuning(Geometry2d::Point goal, float endSpeed) {
    if (!visible) return;

    // sets flags for future movement
    if (verbose)
        cout << " in OurRobot::moveTuning(goal): adding a goal (" << goal.x()
             << ", " << goal.y() << ")" << endl;

    _motionCommand = std::make_unique<Planning::TuningPathCommand>(
        MotionInstant(goal, (goal - pos).normalized() * endSpeed));

    *_cmdText << "moveTuning(" << goal << ")" << endl;
    *_cmdText << "endSpeed(" << endSpeed << ")" << endl;
}

void OurRobot::move(Geometry2d::Point goal, Geometry2d::Point endVelocity) {
    if (!visible) return;

    // sets flags for future movement
    if (verbose)
        cout << " in OurRobot::move(goal): adding a goal (" << goal.x() << ", "
             << goal.y() << ")" << std::endl;

    _motionCommand = std::make_unique<Planning::PathTargetCommand>(
        MotionInstant(goal, endVelocity));

    *_cmdText << "move(" << goal.x() << ", " << goal.y() << ")" << endl;
    *_cmdText << "endVelocity(" << endVelocity.x() << ", " << endVelocity.y()
              << ")" << endl;
}

void OurRobot::lineKick(Point target) {
    if (!visible) return;

    disableAvoidBall();
    _motionCommand =
        std::make_unique<Planning::LineKickCommand>(std::move(target));
}

void OurRobot::worldVelocity(Geometry2d::Point v) {
    _motionCommand = std::make_unique<Planning::WorldVelTargetCommand>(v);
    setPath(nullptr);
    *_cmdText << "worldVel(" << v.x() << ", " << v.y() << ")" << endl;
}

void OurRobot::pivot(Geometry2d::Point pivotTarget) {
    _rotationCommand = std::make_unique<Planning::EmptyAngleCommand>();

    const float radius = Robot_Radius * 1;
    Geometry2d::Point pivotPoint = _state->ball.pos;

    // reset other conflicting motion commands
    _motionCommand = std::make_unique<Planning::PivotCommand>(
        pivotPoint, pivotTarget, radius);

    *_cmdText << "pivot(" << pivotTarget.x() << ", " << pivotTarget.y() << ")"
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

Geometry2d::Point OurRobot::mouthCenterPos() const {
    return kickerBar().center();
}

bool OurRobot::behindBall(Geometry2d::Point ballPos) const {
    Point ballTransformed = pointInRobotSpace(ballPos);
    return ballTransformed.x() < -Robot_Radius;
}

float OurRobot::kickTimer() const {
    return (charged()) ? 0.0 : RJ::numSeconds(RJ::now() - _lastChargedTime);
}

void OurRobot::dribble(uint8_t speed) {
    uint8_t scaled = *config->dribbler.multiplier * speed;
    control->set_dvelocity(scaled);

    *_cmdText << "dribble(" << (float)speed << ")" << endl;
}

void OurRobot::face(Geometry2d::Point pt) {
    _rotationCommand = std::make_unique<Planning::FacePointCommand>(pt);

    *_cmdText << "face(" << pt.x() << ", " << pt.y() << ")" << endl;
}

void OurRobot::faceNone() {
    _rotationCommand = std::make_unique<Planning::EmptyAngleCommand>();

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
    control->set_kcstrength(strength > max ? max : strength);
    control->set_shootmode(Packet::Control::KICK);
    control->set_triggermode(Packet::Control::ON_BREAK_BEAM);
}

void OurRobot::_chip(uint8_t strength) {
    uint8_t max = *config->kicker.maxChip;
    control->set_kcstrength(strength > max ? max : strength);
    control->set_shootmode(Packet::Control::CHIP);
    control->set_triggermode(Packet::Control::ON_BREAK_BEAM);
}

void OurRobot::_unkick() {
    control->set_kcstrength(0);
    control->set_shootmode(Packet::Control::KICK);
    control->set_triggermode(Packet::Control::STAND_DOWN);
}

void OurRobot::unkick() {
    _unkick();

    *_cmdText << "unkick()" << endl;
}

void OurRobot::kickImmediately() {
    control->set_triggermode(Packet::Control::IMMEDIATE);
}

#pragma mark Robot Avoidance

void OurRobot::resetAvoidRobotRadii() {
    for (size_t i = 0; i < Num_Shells; ++i) {
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

#pragma mark Ball Avoidance

void OurRobot::disableAvoidBall() { avoidBallRadius(-1); }

void OurRobot::avoidBallRadius(float radius) {
    _avoidBallRadius = radius;

    *_cmdText << "avoidBall(" << radius << ")" << endl;
}

float OurRobot::avoidBallRadius() const { return _avoidBallRadius; }

void OurRobot::resetAvoidBall() { avoidBallRadius(Ball_Avoid_Small); }

std::shared_ptr<Geometry2d::Circle> OurRobot::createBallObstacle() const {
    // if game is stopped, large obstacle regardless of flags
    if (_state->gameState.state != GameState::Playing &&
        !(_state->gameState.ourRestart || _state->gameState.theirPenalty())) {
        return std::make_shared<Geometry2d::Circle>(
            _state->ball.pos,
            Field_Dimensions::Current_Dimensions.CenterRadius());
    }

    // create an obstacle if necessary
    if (_avoidBallRadius > 0.0) {
        return std::make_shared<Geometry2d::Circle>(_state->ball.pos,
                                                    _avoidBallRadius);
    } else {
        return nullptr;
    }
}

#pragma mark Motion

void OurRobot::setPath(unique_ptr<Planning::Path> path) {
    angleFunctionPath.path = std::move(path);
}

std::vector<Planning::DynamicObstacle> OurRobot::collectDynamicObstacles() {
    vector<Planning::DynamicObstacle> obstacles;

    // Add Opponent Robots
    auto& mask = _opp_avoid_mask;
    auto& robots = _state->opp;
    for (size_t i = 0; i < mask.size(); ++i)
        if (mask[i] > 0 && robots[i] && robots[i]->visible)
            obstacles.push_back(
                Planning::DynamicObstacle(robots[i]->pos, mask[i]));

    // Add ball
    if (_state->ball.valid) {
        auto ballObs = createBallObstacle();
        if (ballObs) obstacles.emplace_back(*ballObs);
    }

    return obstacles;
}

Geometry2d::ShapeSet OurRobot::collectStaticObstacles(
    const Geometry2d::ShapeSet& globalObstacles, bool localObstacles) {
    Geometry2d::ShapeSet fullObstacles{};
    if (localObstacles) {
        fullObstacles = _local_obstacles;
    }

    fullObstacles.add(globalObstacles);

    return fullObstacles;
}

Geometry2d::ShapeSet OurRobot::collectAllObstacles(
    const Geometry2d::ShapeSet& globalObstacles) {
    Geometry2d::ShapeSet fullObstacles(_local_obstacles);
    // Adds our robots as obstacles only if they're within a certain distance
    // from this robot. This distance increases with velocity.
    RobotMask self_avoid_mask;
    std::fill(std::begin(self_avoid_mask), std::end(self_avoid_mask),
              *_selfAvoidRadius);
    const Geometry2d::ShapeSet selfObs = createRobotObstacles(
        _state->self, self_avoid_mask, this->pos, 0.6 + this->vel.mag());
    const Geometry2d::ShapeSet oppObs =
        createRobotObstacles(_state->opp, _opp_avoid_mask);

    if (_state->ball.valid) {
        // _state->drawShape(ball_obs, Qt::gray,
        //                   QString("ball_obstacles_%1").arg(shell()));
        auto ballObs = createBallObstacle();
        if (ballObs) fullObstacles.add(ballObs);
    }
    fullObstacles.add(selfObs);
    fullObstacles.add(oppObs);
    fullObstacles.add(globalObstacles);

    return fullObstacles;
}

bool OurRobot::charged() const {
    return _radioRx.has_kicker_status() && (_radioRx.kicker_status() & 0x01) &&
           rxIsFresh();
}

/*
 * If the ball was recently sensed, then we believe the robot still has it. This
 * avoids noisiness in the ball sensor.
 */
bool OurRobot::hasBall() const {
    if ((RJ::now() - _lastBallSense) < _lostBallDuration) {
        return true;
    }
    return OurRobot::hasBallRaw();
}

bool OurRobot::hasBallRaw() const {
    return _radioRx.has_ball_sense_status() &&
           _radioRx.ball_sense_status() == Packet::HasBall && rxIsFresh();
}

bool OurRobot::ballSenseWorks() const {
    return rxIsFresh() && _radioRx.has_ball_sense_status() &&
           (_radioRx.ball_sense_status() == Packet::NoBall ||
            _radioRx.ball_sense_status() == Packet::HasBall);
}

bool OurRobot::kickerWorks() const {
    return _radioRx.has_kicker_status() &&
           (_radioRx.kicker_status() & Kicker_Enabled) && rxIsFresh();
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
    if (_radioRx.has_quaternion() && rxIsFresh(RJ::Seconds(0.05))) {
        return Eigen::Quaternionf(_radioRx.quaternion().q0() / 16384.0,
                                  _radioRx.quaternion().q1() / 16384.0,
                                  _radioRx.quaternion().q2() / 16384.0,
                                  _radioRx.quaternion().q3() / 16384.0);
    } else {
        return boost::none;
    }
}

bool OurRobot::rxIsFresh(RJ::Seconds age) const {
    return (RJ::now() - RJ::Time(chrono::microseconds(_radioRx.timestamp()))) <
           age;
}

RJ::Timestamp OurRobot::lastKickTime() const {
    return RJ::timestamp(_lastKickTime);
}

void OurRobot::radioRxUpdated() {
    if (_radioRx.kicker_status() < _lastKickerStatus) {
        _lastKickTime = RJ::now();
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

void OurRobot::setPID(double p, double i, double d) {
    config->translation.p->setValueString(QString(std::to_string(p).c_str()));
    config->translation.i->setValueString(QString(std::to_string(i).c_str()));
    config->translation.d->setValueString(QString(std::to_string(d).c_str()));
}
