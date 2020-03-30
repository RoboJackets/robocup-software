#include <Robot.hpp>
#include <LogUtils.hpp>
#include <protobuf/LogFrame.pb.h>
#include <RobotConfig.hpp>
#include <SystemState.hpp>
#include <Utils.hpp>

#include <execinfo.h>
#include <stdio.h>
#include <QString>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>
#include "DebugDrawer.hpp"

using namespace std;
using namespace Geometry2d;
using Planning::MotionInstant;
using Planning::MotionCommand;

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

Robot::Robot(Context* context, unsigned int shell, bool self)
    : _context(context), _shell(shell), _self(self) {}

#pragma mark OurRobot

REGISTER_CONFIGURABLE(OurRobot)

ConfigDouble* OurRobot::_selfAvoidRadius;
ConfigDouble* OurRobot::_oppAvoidRadius;
ConfigDouble* OurRobot::_oppGoalieAvoidRadius;
ConfigDouble* OurRobot::_dribbleOutOfBoundsOffset;

void OurRobot::createConfiguration(Configuration* cfg) {
    _selfAvoidRadius =
        new ConfigDouble(cfg, "PathPlanner/selfAvoidRadius", Robot_Radius);
    _oppAvoidRadius = new ConfigDouble(cfg, "PathPlanner/oppAvoidRadius",
                                       Robot_Radius - 0.01);
    _oppGoalieAvoidRadius = new ConfigDouble(
        cfg, "PathPlanner/oppGoalieAvoidRadius", Robot_Radius + 0.05);
    _dribbleOutOfBoundsOffset = new ConfigDouble(
        cfg, "PathPlanner/dribbleOutOfBoundsOffset", 0.05);
}

OurRobot::OurRobot(Context* context, int shell) : Robot(context, shell, true), _path({}) {
    _cmdText = new std::stringstream();

    //_lastChargedTime = 0;
    _lastKickerStatus = 0;
    //_lastKickTime = 0;
    _lastBallSense = RJ::Time();

    resetAvoidRobotRadii();

    _clearCmdText();
}

OurRobot::~OurRobot() {
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
    dbg->set_layer(_context->debug_drawer.findDebugLayer(layer));
    dbg->set_text(text.toStdString());
    dbg->set_color(color(qc));
    robotText.push_back(dbg);
}

bool OurRobot::avoidOpponents() const {
    // checks for avoiding all opponents
    for (size_t i = 0; i < Num_Shells; ++i) {
        if (_context->state.opp[i] && _context->state.opp[i]->visible() &&
            intent().opp_avoid_mask[i] < 0.1)
            return false;
    }
    return true;
}

void OurRobot::avoidOpponents(bool enable) {
    for (float& a : intent().opp_avoid_mask)
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
    if (verbose && visible())
        cout << "in OurRobot::resetForNextIteration()" << std::endl;
    robotText.clear();

    _clearCmdText();

    intent().clear();
    _context->motion_setpoints[shell()].clear();

    if (charged()) {
        _lastChargedTime = RJ::now();
    }

    intent().local_obstacles.clear();

    resetMotionConstraints();
    _unkick();
    intent().song = RobotIntent::Song::STOP;

    isPenaltyKicker = false;
    isBallPlacer = false;
}

void OurRobot::resetMotionConstraints() {
    _robotConstraints = RobotConstraints();
    _planningPriority = 0;
}

void OurRobot::stop() {
    resetMotionConstraints();

    *_cmdText << "stop()\n";
}

void OurRobot::moveDirect(Geometry2d::Point goal, float endSpeed) {
    if (!visible()) return;

    // sets flags for future movement
    if (verbose)
        cout << " in OurRobot::moveDirect(goal): adding a goal (" << goal.x()
             << ", " << goal.y() << ")" << endl;


    Planning::RobotInstant goal_instant;
    goal_instant.pose = Pose{goal, angle()};
    goal_instant.velocity = Twist{(goal - pos()).normalized() * endSpeed, 0};
    setMotionCommand(std::make_unique<MotionCommand>(Planning::PathTargetCommand{goal_instant}));

    *_cmdText << "moveDirect(" << goal << ")" << endl;
    *_cmdText << "endSpeed(" << endSpeed << ")" << endl;
}

void OurRobot::moveTuning(Geometry2d::Point goal, float endSpeed) {
    if (!visible()) return;

    // sets flags for future movement
    if (verbose)
        cout << " in OurRobot::moveTuning(goal): adding a goal (" << goal.x()
             << ", " << goal.y() << ")" << endl;

    Planning::MotionInstant goal_instant;
    goal_instant.pos = goal;
    goal_instant.vel = (goal - pos()).normalized() * endSpeed;
    setMotionCommand(std::make_unique<MotionCommand>(Planning::TuningPathCommand{goal_instant}));

    *_cmdText << "moveTuning(" << goal << ")" << endl;
    *_cmdText << "endSpeed(" << endSpeed << ")" << endl;
}

void OurRobot::move(Geometry2d::Point goal, Geometry2d::Point endVelocity) {
    if (!visible()) return;

    // sets flags for future movement
    if (verbose)
        cout << " in OurRobot::move(goal): adding a goal (" << goal.x() << ", "
             << goal.y() << ")" << std::endl;

    Planning::RobotInstant goal_instant;
    //todo(Ethan) fix this angle
    goal_instant.pose = Pose{goal, angle()};
    goal_instant.velocity = Twist{endVelocity, 0};
    setMotionCommand(std::make_unique<MotionCommand>(Planning::PathTargetCommand{goal_instant}));

    *_cmdText << "move(" << goal.x() << ", " << goal.y() << ")" << endl;
    *_cmdText << "endVelocity(" << endVelocity.x() << ", " << endVelocity.y()
              << ")" << endl;
}

void OurRobot::settle() {
    if (!visible()) return;
    setMotionCommand(std::make_unique<MotionCommand>(Planning::SettleCommand{}));
}

void OurRobot::collect() {
    if (!visible()) return;
    setMotionCommand(std::make_unique<MotionCommand>(Planning::CollectCommand{}));
}

void OurRobot::lineKick(Point target) {
    if (!visible()) return;

    disableAvoidBall();
    setMotionCommand(std::make_unique<MotionCommand>(Planning::LineKickCommand{target}));
}

void OurRobot::intercept(Point target) {
    if (!visible()) return;
    //todo(Ethan) add the InterceptPlanner
    disableAvoidBall();
    setMotionCommand(std::make_unique<MotionCommand>(Planning::InterceptCommand{target}));
}

void OurRobot::worldVelocity(Geometry2d::Point v) {
    setMotionCommand(std::make_unique<MotionCommand>(Planning::WorldVelTargetCommand{Twist{v, 0}}));
    *_cmdText << "worldVel(" << v.x() << ", " << v.y() << ")" << endl;
}

void OurRobot::pivot(Geometry2d::Point pivotTarget) {
    Geometry2d::Point pivotPoint = _context->state.ball.pos;

    // reset other conflicting motion commands
    setMotionCommand(std::make_unique<MotionCommand>(Planning::PivotCommand{pivotPoint, pivotTarget}));

    *_cmdText << "pivot(" << pivotTarget.x() << ", " << pivotTarget.y() << ")"
              << endl;
}

Geometry2d::Point OurRobot::pointInRobotSpace(Geometry2d::Point pt) const {
    Point p = pt;
    p.rotate(pos(), -angle());
    return p;
}

const Geometry2d::Segment OurRobot::kickerBar() const {
    TransformMatrix pose(pos(), angle());
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

// TODO make speed a float from 0->1 to make this more clear.
void OurRobot::dribble(uint8_t speed) {
    Field_Dimensions current_dimensions = Field_Dimensions::Current_Dimensions;
    float offset = *_dribbleOutOfBoundsOffset;

    Geometry2d::Rect modifiedField = Geometry2d::Rect(
        Point((-current_dimensions.Width() / 2) - offset, -offset),
        Point((current_dimensions.Width() / 2) + offset,
              current_dimensions.Length() + offset));

    if (modifiedField.containsPoint(pos())) {
        uint8_t scaled = std::min(*config->dribbler.multiplier * speed, (double) Max_Dribble);
        intent().dvelocity = scaled;

        *_cmdText << "dribble(" << (float)speed << ")" << endl;
    } else {
        intent().dvelocity = 0;
    }
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
    intent().kcstrength = (strength > max ? max : strength);
    intent().shoot_mode = RobotIntent::ShootMode::KICK;
    intent().trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
}

void OurRobot::_chip(uint8_t strength) {
    uint8_t max = *config->kicker.maxChip;
    intent().kcstrength = (strength > max ? max : strength);
    intent().shoot_mode = RobotIntent::ShootMode::CHIP;
    intent().trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
}

void OurRobot::_unkick() {
    intent().kcstrength = 0;
    intent().shoot_mode = RobotIntent::ShootMode::KICK;
    intent().trigger_mode = RobotIntent::TriggerMode::STAND_DOWN;
}

void OurRobot::unkick() {
    _unkick();

    *_cmdText << "unkick()" << endl;
}

void OurRobot::kickImmediately() {
    intent().trigger_mode = RobotIntent::TriggerMode::IMMEDIATE;
}

void OurRobot::face(Geometry2d::Point pt) {
    intent().angle_override = pos().angleTo(pt);
}
bool OurRobot::isFacing() const {
    return !intent().angle_override;
}
void OurRobot::faceNone() {
    intent().angle_override = std::nullopt;
}

#pragma mark Robot Avoidance

void OurRobot::resetAvoidRobotRadii() {
    for (size_t i = 0; i < Num_Shells; ++i) {
        intent().opp_avoid_mask[i] =
            (i == _context->game_state.TheirInfo.goalie)
                ? *_oppGoalieAvoidRadius
                : *_oppAvoidRadius;
    }
}

void OurRobot::approachAllOpponents(bool enable) {
    for (float& ar : intent().opp_avoid_mask)
        ar = (enable) ? Opp_Avoid_Small : *_oppAvoidRadius;
}
void OurRobot::avoidAllOpponents(bool enable) {
    for (float& ar : intent().opp_avoid_mask)
        ar = (enable) ? -1.0 : *_oppAvoidRadius;
}

bool OurRobot::avoidOpponent(unsigned shell_id) const {
    return intent().opp_avoid_mask[shell_id] > 0.0;
}

bool OurRobot::approachOpponent(unsigned shell_id) const {
    return avoidOpponent(shell_id) &&
           intent().opp_avoid_mask[shell_id] < Robot_Radius - 0.01;
}

float OurRobot::avoidOpponentRadius(unsigned shell_id) const {
    return intent().opp_avoid_mask[shell_id];
}

void OurRobot::avoidOpponent(unsigned shell_id, bool enable_avoid) {
    if (enable_avoid)
        intent().opp_avoid_mask[shell_id] = *_oppAvoidRadius;
    else
        intent().opp_avoid_mask[shell_id] = -1.0;
}

void OurRobot::approachOpponent(unsigned shell_id, bool enable_approach) {
    if (enable_approach)
        intent().opp_avoid_mask[shell_id] = Opp_Avoid_Small;
    else
        intent().opp_avoid_mask[shell_id] = *_oppAvoidRadius;
}

void OurRobot::avoidOpponentRadius(unsigned shell_id, float radius) {
    intent().opp_avoid_mask[shell_id] = radius;
}

void OurRobot::avoidAllOpponentRadius(float radius) {
    for (float& ar : intent().opp_avoid_mask) ar = radius;
}

#pragma mark Ball Avoidance

void OurRobot::disableAvoidBall() { avoidBallRadius(-1); }

void OurRobot::avoidBallRadius(float radius) {
    intent().avoid_ball_radius = radius;

    *_cmdText << "avoidBall(" << radius << ")" << endl;
}

float OurRobot::avoidBallRadius() const { return intent().avoid_ball_radius; }

void OurRobot::resetAvoidBall() { avoidBallRadius(Ball_Avoid_Small); }

std::shared_ptr<Geometry2d::Circle> OurRobot::createBallObstacle() const {
    // if game is stopped, large obstacle regardless of flags
    if (_context->game_state.state != GameState::Playing &&
        !(_context->game_state.ourRestart ||
          _context->game_state.theirPenalty())) {
        return std::make_shared<Geometry2d::Circle>(
            _context->state.ball.pos,
            Field_Dimensions::Current_Dimensions.CenterRadius());
    }

    // create an obstacle if necessary
    if (intent().avoid_ball_radius > 0.0) {
        return std::make_shared<Geometry2d::Circle>(_context->state.ball.pos,
                                                    intent().avoid_ball_radius);
    } else {
        return nullptr;
    }
}

#pragma mark Motion

void OurRobot::setPath(Planning::Trajectory&& new_path) {
    this->_path = std::move(new_path);
}

Geometry2d::ShapeSet OurRobot::collectStaticObstacles(
    const Geometry2d::ShapeSet& globalObstacles, bool localObstacles) {
    Geometry2d::ShapeSet fullObstacles{};
    if (localObstacles) {
        fullObstacles = intent().local_obstacles;
    }

    // Add Opponent Robots
    auto& mask = intent().opp_avoid_mask;
    auto& robots = _context->state.opp;
    for (size_t i = 0; i < mask.size(); ++i) {
        if (mask[i] > 0 && robots[i] && robots[i]->visible()) {
            fullObstacles.add(std::make_shared<Circle>(robots[i]->pos(), mask[i]));
        }
    }

    // Add ball
    bool possessBall = _context->ball_possessor && *_context->ball_possessor == shell();
    if (_context->state.ball.valid && !possessBall) {
        auto ballObs = createBallObstacle();
        if (ballObs) fullObstacles.add(ballObs);
    }

    fullObstacles.add(globalObstacles);

    return fullObstacles;
}

Geometry2d::ShapeSet OurRobot::collectAllObstacles(
    const Geometry2d::ShapeSet& globalObstacles) {
    Geometry2d::ShapeSet fullObstacles(intent().local_obstacles);
    // Adds our robots as obstacles only if they're within a certain distance
    // from this robot. This distance increases with velocity.
    RobotMask self_avoid_mask;
    std::fill(std::begin(self_avoid_mask), std::end(self_avoid_mask),
              *_selfAvoidRadius);
    const Geometry2d::ShapeSet selfObs =
        createRobotObstacles(_context->state.self, self_avoid_mask, this->pos(),
                             0.6 + this->vel().mag());
    const Geometry2d::ShapeSet oppObs =
        createRobotObstacles(_context->state.opp, intent().opp_avoid_mask);

    if (_context->state.ball.valid) {
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
    return kickerWorks() && *status->chipper_enabled;
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

std::optional<Eigen::Quaternionf> OurRobot::quaternion() const {
    if (_radioRx.has_quaternion() && rxIsFresh(RJ::Seconds(0.05))) {
        return Eigen::Quaternionf(_radioRx.quaternion().q0() / 16384.0,
                                  _radioRx.quaternion().q1() / 16384.0,
                                  _radioRx.quaternion().q2() / 16384.0,
                                  _radioRx.quaternion().q3() / 16384.0);
    } else {
        return std::nullopt;
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

void OurRobot::setJoystickControlled(bool joystickControlled) {
    _joystickControlled = joystickControlled;
}

bool OurRobot::isJoystickControlled() const { return _joystickControlled; }
