#include <cmath>
#include <iostream>
#include <stdexcept>

#include <QString>

#include <Robot.hpp>
#include <RobotConfig.hpp>
#include <SystemState.hpp>

#include "DebugDrawer.hpp"

using namespace std;
using namespace Geometry2d;
using Planning::LinearMotionInstant;
using Planning::MotionCommand;

/** thresholds for avoidance of opponents - either a normal (large) or an
 * approach (small)*/
constexpr float kOppAvoidSmallThresh = 0.03;
constexpr float kOppAvoidSmall = Robot_Radius - kOppAvoidSmallThresh;
/** threshold for avoiding the ball*/
constexpr float kBallAvoidSmallMult = 2.0;
constexpr float kBallAvoidSmall = kBallAvoidSmallMult * Ball_Radius;
/**
 * When verbose is true, a lot of extra debug info is printed
 * to the console about path planning, etc
 */
constexpr bool kVerbose = false;

Robot::Robot(Context* context, int shell, bool self)
    : _context(context), _shell(shell), _self(self) {}

#pragma mark OurRobot

REGISTER_CONFIGURABLE(OurRobot)

ConfigDouble* OurRobot::self_avoid_radius;
ConfigDouble* OurRobot::opp_avoid_radius;
ConfigDouble* OurRobot::opp_goalie_avoid_radius;
ConfigDouble* OurRobot::dribble_out_of_bounds_offset;

void OurRobot::createConfiguration(Configuration* cfg) {
    self_avoid_radius =
        new ConfigDouble(cfg, "PathPlanner/selfAvoidRadius", Robot_Radius);
    opp_avoid_radius = new ConfigDouble(cfg, "PathPlanner/oppAvoidRadius",
                                        Robot_Radius - 0.01);
    opp_goalie_avoid_radius = new ConfigDouble(
        cfg, "PathPlanner/oppGoalieAvoidRadius", Robot_Radius + 0.05);
    dribble_out_of_bounds_offset =  // NOLINT
        new ConfigDouble(cfg, "PathPlanner/dribbleOutOfBoundsOffset", 0.05);
}

OurRobot::OurRobot(Context* context, int shell) : Robot(context, shell, true) {
    resetAvoidRobotRadii();

    _clearCmdText();
}

void OurRobot::addStatusText() {
    const QColor status_color(255, 32, 32);

    if (!statusIsFresh()) {
        addText("No RX", status_color, "Status");
    }
}

void OurRobot::addText(const QString& text, const QColor& qc,
                       const QString& layer_prefix) {
    QString layer = layer_prefix + QString::number(shell());
    _context->debug_drawer.drawText(text, pos(), qc, layer);
}

bool OurRobot::avoidOpponents() const {
    // checks for avoiding all opponents
    for (size_t i = 0; i < Num_Shells; ++i) {
        if ((_context->state.opp[i] != nullptr) &&
            _context->state.opp[i]->visible() &&
            intent().opp_avoid_mask[i] < 0.1) {
            return false;
        }
    }
    return true;
}

void OurRobot::avoidOpponents(bool enable) {
    for (float& a : intent().opp_avoid_mask) {
        if (enable) {
            a = Robot_Radius - 0.03;
        } else {
            a = -1.0;
        }
    }
}

std::string OurRobot::getCmdText() const { return _cmdText.str(); }

void OurRobot::_clearCmdText() {
    _cmdText.str("");
    _cmdText.clear();
}

void OurRobot::resetForNextIteration() {
    if (kVerbose && visible()) {
        cout << "in OurRobot::resetForNextIteration()" << std::endl;
    }

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
    robotConstraints() = RobotConstraints();
    _planningPriority = 0;
}

void OurRobot::stop() {
    resetMotionConstraints();

    _cmdText << "stop()\n";
}

void OurRobot::moveDirect(Geometry2d::Point goal, float end_speed) {
    if (!visible()) {
        return;
    }

    // sets flags for future movement
    if (kVerbose) {
        cout << " in OurRobot::moveDirect(goal): adding a goal (" << goal.x()
             << ", " << goal.y() << ")" << endl;
    }

    LinearMotionInstant goal_instant;
    goal_instant.position = goal;
    goal_instant.velocity = (goal - pos()).normalized(end_speed);
    setMotionCommand(Planning::PathTargetCommand{goal_instant});

    _cmdText << "moveDirect(" << goal << ")" << endl;
    _cmdText << "endSpeed(" << end_speed << ")" << endl;
}

void OurRobot::moveTuning(Geometry2d::Point goal, float end_speed) {
    if (!visible()) {
        return;
    }

    // sets flags for future movement
    if (kVerbose) {
        cout << " in OurRobot::moveTuning(goal): adding a goal (" << goal.x()
             << ", " << goal.y() << ")" << endl;
    }

    // TODO(#1510): Add in a tuning planner.
    Geometry2d::Point target_point = goal;
    Geometry2d::Point target_vel = (goal - pos()).normalized() * end_speed;
    LinearMotionInstant goal_instant{target_point, target_vel};
    setMotionCommand(Planning::PathTargetCommand{goal_instant});

    _cmdText << "moveTuning(" << goal << ")" << endl;
    _cmdText << "endSpeed(" << end_speed << ")" << endl;
}

void OurRobot::move(Geometry2d::Point goal, Geometry2d::Point end_velocity) {
    if (!visible()) {
        return;
    }

    // sets flags for future movement
    if (kVerbose) {
        cout << " in OurRobot::move(goal): adding a goal (" << goal.x() << ", "
             << goal.y() << ")" << std::endl;
    }

    Planning::AngleOverride angle_override = Planning::TargetFaceTangent{};
    if (std::holds_alternative<Planning::PathTargetCommand>(
            intent().motion_command)) {
        angle_override =
            std::get<Planning::PathTargetCommand>(intent().motion_command)
                .angle_override;
    }

    LinearMotionInstant goal_instant{goal, end_velocity};
    setMotionCommand(Planning::PathTargetCommand{goal_instant, angle_override});

    _cmdText << "move(" << goal.x() << ", " << goal.y() << ")" << endl;
    _cmdText << "endVelocity(" << end_velocity.x() << ", " << end_velocity.y()
             << ")" << endl;
}

void OurRobot::settle(std::optional<Point> target) {
    if (!visible()) {
        return;
    }

    // NOLINTNEXTLINE: the CI clang-tidy version (wrongly) suggests std::move
    setMotionCommand(Planning::SettleCommand{target});
}

void OurRobot::collect() {
    if (!visible()) {
        return;
    }

    setMotionCommand(Planning::CollectCommand{});
}

void OurRobot::lineKick(Point target) {
    if (!visible()) {
        return;
    }

    disableAvoidBall();
    setMotionCommand(Planning::LineKickCommand{target});
}

void OurRobot::intercept(Point target) {
    if (!visible()) {
        return;
    }
    disableAvoidBall();
    setMotionCommand(Planning::InterceptCommand{target});
}

void OurRobot::worldVelocity(Geometry2d::Point target_world_vel) {
    setMotionCommand(Planning::WorldVelCommand{target_world_vel});
    _cmdText << "worldVel(" << target_world_vel.x() << ", "
             << target_world_vel.y() << ")" << endl;
}

void OurRobot::pivot(Geometry2d::Point pivot_target) {
    Geometry2d::Point pivot_point = _context->world_state.ball.position;

    // reset other conflicting motion commands
    setMotionCommand(Planning::PivotCommand{pivot_point, pivot_target});

    _cmdText << "pivot(" << pivot_target.x() << ", " << pivot_target.y() << ")"
             << endl;
}

Geometry2d::Point OurRobot::pointInRobotSpace(Geometry2d::Point pt) const {
    Point p = pt;
    p.rotate(pos(), -angle());
    return p;
}

Geometry2d::Segment OurRobot::kickerBar() const {
    TransformMatrix pose(pos(), static_cast<float>(angle()));
    const float mouth_half = Robot_MouthWidth / 2.0f;
    float x = sin(acos(mouth_half / Robot_Radius)) * Robot_Radius;
    Point l(x, Robot_MouthWidth / 2.0f);
    Point r(x, -Robot_MouthWidth / 2.0f);
    return Segment(pose * l, pose * r);
}

Geometry2d::Point OurRobot::mouthCenterPos() const {
    return kickerBar().center();
}

bool OurRobot::behindBall(Geometry2d::Point ball_pos) const {
    Point ball_transformed = pointInRobotSpace(ball_pos);
    return ball_transformed.x() < -Robot_Radius;
}

float OurRobot::kickTimer() const {
    return (charged()) ? 0.0f
                       : static_cast<float>(
                             RJ::numSeconds(RJ::now() - _lastChargedTime));
}

// TODO make speed a float from 0->1 to make this more clear.
void OurRobot::dribble(uint8_t speed) {
    Field_Dimensions current_dimensions = Field_Dimensions::Current_Dimensions;
    auto offset = static_cast<float>(*dribble_out_of_bounds_offset);

    Geometry2d::Rect modified_field = Geometry2d::Rect(
        Point((-current_dimensions.Width() / 2) - offset, -offset),
        Point((current_dimensions.Width() / 2) + offset,
              current_dimensions.Length() + offset));

    if (modified_field.containsPoint(pos())) {
        uint8_t scaled = std::min(*config()->dribbler.multiplier * speed,
                                  static_cast<double>(Max_Dribble));
        intent().dvelocity = scaled;

        _cmdText << "dribble(" << (float)speed << ")" << endl;
    } else {
        intent().dvelocity = 0;
    }
}

void OurRobot::kick(float strength) {
    double max_kick = *config()->kicker.maxKick;
    _kick(roundf(strength * ((float)max_kick)));

    _cmdText << "kick(" << strength * 100 << "%)" << endl;
}

void OurRobot::kickLevel(uint8_t strength) {
    _kick(strength);

    _cmdText << "kick(" << (float)strength << ")" << endl;
}

void OurRobot::chip(float strength) {
    double max_chip = *config()->kicker.maxChip;
    _chip(roundf(strength * ((float)max_chip)));
    _cmdText << "chip(" << strength * 100 << "%)" << endl;
}

void OurRobot::chipLevel(uint8_t strength) {
    _chip(strength);

    _cmdText << "chip(" << (float)strength << ")" << endl;
}

void OurRobot::_kick(uint8_t strength) {
    uint8_t max = *config()->kicker.maxKick;
    intent().kcstrength = (strength > max ? max : strength);
    intent().shoot_mode = RobotIntent::ShootMode::KICK;
    intent().trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
}

void OurRobot::_chip(uint8_t strength) {
    uint8_t max = *config()->kicker.maxChip;
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

    _cmdText << "unkick()" << endl;
}

void OurRobot::kickImmediately() {
    intent().trigger_mode = RobotIntent::TriggerMode::IMMEDIATE;
}

void OurRobot::face(Geometry2d::Point pt) {
    if (!std::holds_alternative<Planning::PathTargetCommand>(
            intent().motion_command)) {
        intent().motion_command.emplace<Planning::PathTargetCommand>();
    }

    _cmdText << "face(" << pt << ")" << endl;

    auto& command =
        std::get<Planning::PathTargetCommand>(intent().motion_command);
    command.angle_override = Planning::TargetFacePoint{pt};
}
#pragma mark Robot Avoidance

void OurRobot::resetAvoidRobotRadii() {
    for (size_t i = 0; i < Num_Shells; ++i) {
        intent().opp_avoid_mask[i] = (i == _context->their_info.goalie)
                                         ? *opp_goalie_avoid_radius
                                         : *opp_avoid_radius;
    }
}

void OurRobot::approachAllOpponents(bool enable) {
    for (float& ar : intent().opp_avoid_mask) {
        ar = (enable) ? kOppAvoidSmall : static_cast<float>(*opp_avoid_radius);
    }
}
void OurRobot::avoidAllOpponents(bool enable) {
    for (float& ar : intent().opp_avoid_mask) {
        ar = (enable) ? -1.0f : static_cast<float>(*opp_avoid_radius);
    }
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
    if (enable_avoid) {
        intent().opp_avoid_mask[shell_id] = *opp_avoid_radius;
    } else {
        intent().opp_avoid_mask[shell_id] = -1.0;
    }
}

void OurRobot::approachOpponent(unsigned shell_id, bool enable_approach) {
    if (enable_approach) {
        intent().opp_avoid_mask[shell_id] = kOppAvoidSmall;
    } else {
        intent().opp_avoid_mask[shell_id] = *opp_avoid_radius;
    }
}

void OurRobot::avoidOpponentRadius(unsigned shell_id, float radius) {
    intent().opp_avoid_mask[shell_id] = radius;
}

void OurRobot::avoidAllOpponentRadius(float radius) {
    for (float& ar : intent().opp_avoid_mask) {
        ar = radius;
    }
}

#pragma mark Ball Avoidance

void OurRobot::disableAvoidBall() { avoidBallRadius(-1); }

void OurRobot::avoidBallRadius(float radius) {
    intent().avoid_ball_radius = radius;

    _cmdText << "avoidBall(" << radius << ")" << endl;
}

float OurRobot::avoidBallRadius() const { return intent().avoid_ball_radius; }

void OurRobot::resetAvoidBall() { avoidBallRadius(kBallAvoidSmall); }

std::shared_ptr<Geometry2d::Circle> OurRobot::createBallObstacle() const {
    // if game is stopped, large obstacle regardless of flags
    if (_context->game_state.state != GameState::Playing &&
        !(_context->game_state.our_restart ||
          _context->game_state.theirPenalty())) {
        return std::make_shared<Geometry2d::Circle>(
            _context->world_state.ball.position,
            Field_Dimensions::Current_Dimensions.CenterRadius());
    }

    // create an obstacle if necessary
    if (intent().avoid_ball_radius > 0.0) {
        return std::make_shared<Geometry2d::Circle>(
            _context->world_state.ball.position, intent().avoid_ball_radius);
    }
    return nullptr;
}

#pragma mark Motion

bool OurRobot::charged() const {
    return radioStatus().kicker == RobotStatus::KickerState::kCharged &&
           statusIsFresh();
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
    return radioStatus().has_ball && statusIsFresh();
}

bool OurRobot::ballSenseWorks() const {
    // TODO(Kyle): Get error code from robot.
    return statusIsFresh();
}

bool OurRobot::kickerWorks() const {
    return radioStatus().kicker != RobotStatus::KickerState::kFailed &&
           statusIsFresh();
}

bool OurRobot::chipper_available() const {
    return kickerWorks() && *status()->chipper_enabled;
}

bool OurRobot::kicker_available() const {
    return kickerWorks() && *status()->kicker_enabled;
}

bool OurRobot::dribbler_available() const {
    return *status()->dribbler_enabled && radioStatus().motors_healthy[4] &&
           statusIsFresh();
}

bool OurRobot::driving_available(bool require_all) const {
    if (!statusIsFresh()) {
        return false;
    }

    int c = 0;
    for (int i = 0; i < 4; ++i) {
        if (radioStatus().motors_healthy[i]) {
            ++c;
        }
    }

    return require_all ? (c == 4) : (c == 3);
}

double OurRobot::kickerVoltage() const {
    if (!statusIsFresh()) {
        return 0;
    }

    return radioStatus().kicker_voltage;
}

RobotStatus::HardwareVersion OurRobot::hardwareVersion() const {
    if (statusIsFresh()) {
        return radioStatus().version;
    }
    return RobotStatus::HardwareVersion::kUnknown;
}

bool OurRobot::statusIsFresh(RJ::Seconds age) const {
    return (RJ::now() - radioStatus().timestamp) < age;
}

RJ::Timestamp OurRobot::lastKickTime() const {
    return RJ::timestamp(_lastKickTime);
}

void OurRobot::radioRxUpdated() {
    if (radioStatus().kicker == RobotStatus::KickerState::kCharging &&
        _lastKickerStatus == RobotStatus::KickerState::kCharged) {
        _lastKickTime = RJ::now();
    }
    _lastKickerStatus = radioStatus().kicker;
}

double OurRobot::distanceToChipLanding(int chip_power) {
    return max(0.,
               min(190., *(config()->chipper.calibrationSlope) * chip_power +
                             *(config()->chipper.calibrationOffset)));
}

uint8_t OurRobot::chipPowerForDistance(double distance) {
    double b = *(config()->chipper.calibrationOffset) / 2.;
    if (distance < b) {
        return 0;
    }
    if (distance > distanceToChipLanding(255)) {
        return 255;
    }
    return 0.5 * distance + b;
}

void OurRobot::setPID(double p, double i, double d) {
    config()->translation.p->setValueString(QString(std::to_string(p).c_str()));
    config()->translation.i->setValueString(QString(std::to_string(i).c_str()));
    config()->translation.d->setValueString(QString(std::to_string(d).c_str()));
}

void OurRobot::setJoystickControlled(bool joystick_controlled) {
    _context->is_joystick_controlled[shell()] = joystick_controlled;
}

bool OurRobot::isJoystickControlled() const {
    return _context->is_joystick_controlled[shell()];
}
