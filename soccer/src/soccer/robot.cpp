#include <cmath>
#include <stdexcept>

#include <QString>

#include <robot.hpp>
#include <robot_config.hpp>
#include <system_state.hpp>

#include "debug_drawer.hpp"

using namespace std;
using namespace rj_geometry;
using Planning::LinearMotionInstant;
using Planning::MotionCommand;

// TODO(Kyle): Make these ROS parameters and move them to a central location
constexpr double kMaxKickSpeed = 7.0;
constexpr double kMinKickSpeed = 2.1;

static double kicker_strength_to_speed(uint8_t kick_strength) {
    return std::min(1.0, static_cast<double>(kick_strength) / kMaxKick) *
               (kMaxKickSpeed - kMinKickSpeed) +
           kMinKickSpeed;
}

/**
 * When verbose is true, a lot of extra debug info is printed
 * to the console about path planning, etc
 */
constexpr bool kVerbose = false;

Robot::Robot(Context* context, int shell, bool self)
    : context_(context), shell_(shell), self_(self) {}

#pragma mark OurRobot

REGISTER_CONFIGURABLE(OurRobot)

ConfigDouble* OurRobot::dribble_out_of_bounds_offset;

void OurRobot::create_configuration(Configuration* cfg) {
    dribble_out_of_bounds_offset =  // NOLINT
        new ConfigDouble(cfg, "PathPlanner/dribbleOutOfBoundsOffset", 0.05);
}

OurRobot::OurRobot(Context* context, int shell) : Robot(context, shell, true) {
    clear_cmd_text();
}

void OurRobot::add_status_text() {
    const QColor status_color(255, 32, 32);

    if (!status_is_fresh()) {
        add_text("No RX", status_color, "Status");
    }
}

void OurRobot::add_text(const QString& text, const QColor& qc, const QString& layer_prefix) {
    QString layer = layer_prefix + QString::number(shell());
    context_->debug_drawer.draw_text(text, pos(), qc, layer);
}

std::string OurRobot::get_cmd_text() const { return cmd_text_.str(); }

void OurRobot::clear_cmd_text() {
    cmd_text_.str("");
    cmd_text_.clear();
}

void OurRobot::reset_for_next_iteration() {
    if (kVerbose && visible()) {
        cout << "in OurRobot::reset_for_next_iteration()" << std::endl;
    }

    clear_cmd_text();

    intent() = {};
    context_->motion_setpoints[shell()] = {};

    if (charged()) {
        last_charged_time_ = RJ::now();
    }

    intent().local_obstacles.clear();

    reset_motion_constraints();
    do_unkick();

    is_penalty_kicker = false;
    is_ball_placer = false;
}

void OurRobot::reset_motion_constraints() {
    robot_constraints() = RobotConstraints();
    planning_priority_ = 0;
}

void OurRobot::stop() {
    reset_motion_constraints();

    cmd_text_ << "stop()\n";
}

void OurRobot::move_direct(rj_geometry::Point goal, float end_speed) {
    if (!visible()) {
        return;
    }

    // sets flags for future movement
    if (kVerbose) {
        cout << " in OurRobot::move_direct(goal): adding a goal (" << goal.x() << ", " << goal.y()
             << ")" << endl;
    }

    LinearMotionInstant goal_instant;
    goal_instant.position = goal;
    goal_instant.velocity = (goal - pos()).normalized(end_speed);
    set_motion_command(Planning::PathTargetCommand{goal_instant});

    cmd_text_ << "move_direct(" << goal << ")" << endl;
    cmd_text_ << "end_speed(" << end_speed << ")" << endl;
}

void OurRobot::move_tuning(rj_geometry::Point goal, float end_speed) {
    if (!visible()) {
        return;
    }

    // sets flags for future movement
    if (kVerbose) {
        cout << " in OurRobot::move_tuning(goal): adding a goal (" << goal.x() << ", " << goal.y()
             << ")" << endl;
    }

    // TODO(#1510): Add in a tuning planner.
    rj_geometry::Point target_point = goal;
    rj_geometry::Point target_vel = (goal - pos()).normalized() * end_speed;
    LinearMotionInstant goal_instant{target_point, target_vel};
    set_motion_command(Planning::PathTargetCommand{goal_instant});

    cmd_text_ << "move_tuning(" << goal << ")" << endl;
    cmd_text_ << "end_speed(" << end_speed << ")" << endl;
}

void OurRobot::move(rj_geometry::Point goal, rj_geometry::Point end_velocity) {
    if (!visible()) {
        return;
    }

    // sets flags for future movement
    if (kVerbose) {
        cout << " in OurRobot::move(goal): adding a goal (" << goal.x() << ", " << goal.y() << ")"
             << std::endl;
    }

    Planning::AngleOverride angle_override = Planning::TargetFaceTangent{};
    if (std::holds_alternative<Planning::PathTargetCommand>(intent().motion_command)) {
        angle_override =
            std::get<Planning::PathTargetCommand>(intent().motion_command).angle_override;
    }

    LinearMotionInstant goal_instant{goal, end_velocity};
    set_motion_command(Planning::PathTargetCommand{goal_instant, angle_override});

    cmd_text_ << "move(" << goal.x() << ", " << goal.y() << ")" << endl;
    cmd_text_ << "end_velocity(" << end_velocity.x() << ", " << end_velocity.y() << ")" << endl;
}

void OurRobot::settle(std::optional<Point> target) {
    if (!visible()) {
        return;
    }

    // NOLINTNEXTLINE: the CI clang-tidy version (wrongly) suggests std::move
    set_motion_command(Planning::SettleCommand{target});
}

void OurRobot::collect() {
    if (!visible()) {
        return;
    }

    set_motion_command(Planning::CollectCommand{});
}

void OurRobot::line_kick(Point target) {
    if (!visible()) {
        return;
    }

    set_motion_command(Planning::LineKickCommand{target});
}

void OurRobot::intercept(Point target) {
    if (!visible()) {
        return;
    }

    set_motion_command(Planning::InterceptCommand{target});
}

void OurRobot::world_velocity(rj_geometry::Point target_world_vel) {
    set_motion_command(Planning::WorldVelCommand{target_world_vel});
    cmd_text_ << "world_vel(" << target_world_vel.x() << ", " << target_world_vel.y() << ")"
              << endl;
}

void OurRobot::pivot(rj_geometry::Point pivot_target) {
    rj_geometry::Point pivot_point = context_->world_state.ball.position;

    // reset other conflicting motion commands
    set_motion_command(Planning::PivotCommand{pivot_point, pivot_target});

    cmd_text_ << "pivot(" << pivot_target.x() << ", " << pivot_target.y() << ")" << endl;
}

rj_geometry::Point OurRobot::point_in_robot_space(rj_geometry::Point pt) const {
    Point p = pt;
    p.rotate(pos(), -angle());
    return p;
}

rj_geometry::Segment OurRobot::kicker_bar() const {
    TransformMatrix pose(pos(), static_cast<float>(angle()));
    const float mouth_half = kRobotMouthWidth / 2.0f;
    float x = sin(acos(mouth_half / kRobotRadius)) * kRobotRadius;
    Point l(x, kRobotMouthWidth / 2.0f);
    Point r(x, -kRobotMouthWidth / 2.0f);
    return Segment(pose * l, pose * r);
}

rj_geometry::Point OurRobot::mouth_center_pos() const { return kicker_bar().center(); }

bool OurRobot::behind_ball(rj_geometry::Point ball_pos) const {
    Point ball_transformed = point_in_robot_space(ball_pos);
    return ball_transformed.x() < -kRobotRadius;
}

float OurRobot::kick_timer() const {
    return (charged()) ? 0.0f : static_cast<float>(RJ::num_seconds(RJ::now() - last_charged_time_));
}

// TODO make speed a float from 0->1 to make this more clear.
void OurRobot::dribble(uint8_t speed) {
    FieldDimensions current_dimensions = FieldDimensions::current_dimensions;
    auto offset = static_cast<float>(*dribble_out_of_bounds_offset);

    rj_geometry::Rect modified_field = rj_geometry::Rect(
        Point((-current_dimensions.width() / 2) - offset, -offset),
        Point((current_dimensions.width() / 2) + offset, current_dimensions.length() + offset));

    if (modified_field.contains_point(pos())) {
        uint8_t scaled =
            std::min(*config()->dribbler.multiplier * speed, static_cast<double>(kMaxDribble));
        intent().dribbler_speed = scaled;

        cmd_text_ << "dribble(" << (float)speed << ")" << endl;
    } else {
        intent().dribbler_speed = 0;
    }
}

void OurRobot::kick(float strength) {
    double max_kick = *config()->kicker.max_kick;
    do_kick(roundf(strength * ((float)max_kick)));

    cmd_text_ << "kick(" << strength * 100 << "%)" << endl;
}

void OurRobot::kick_level(uint8_t strength) {
    do_kick(strength);

    cmd_text_ << "kick(" << (float)strength << ")" << endl;
}

void OurRobot::chip(float strength) {
    double max_chip = *config()->kicker.max_chip;
    do_chip(roundf(strength * ((float)max_chip)));
    cmd_text_ << "chip(" << strength * 100 << "%)" << endl;
}

void OurRobot::chip_level(uint8_t strength) {
    do_chip(strength);

    cmd_text_ << "chip(" << (float)strength << ")" << endl;
}

void OurRobot::do_kick(uint8_t strength) {
    uint8_t max = *config()->kicker.max_kick;
    strength = (strength > max ? max : strength);
    intent().kick_speed = kicker_strength_to_speed(strength);
    intent().shoot_mode = RobotIntent::ShootMode::KICK;
    intent().trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
}

void OurRobot::do_chip(uint8_t strength) {
    uint8_t max = *config()->kicker.max_chip;
    strength = (strength > max ? max : strength);
    intent().kick_speed = kicker_strength_to_speed(strength);
    intent().shoot_mode = RobotIntent::ShootMode::CHIP;
    intent().trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
}

void OurRobot::do_unkick() {
    intent().kick_speed = 0;
    intent().shoot_mode = RobotIntent::ShootMode::KICK;
    intent().trigger_mode = RobotIntent::TriggerMode::STAND_DOWN;
}

void OurRobot::unkick() {
    do_unkick();

    cmd_text_ << "unkick()" << endl;
}

void OurRobot::kick_immediately() { intent().trigger_mode = RobotIntent::TriggerMode::IMMEDIATE; }

void OurRobot::face(rj_geometry::Point pt) {
    if (!std::holds_alternative<Planning::PathTargetCommand>(intent().motion_command)) {
        intent().motion_command.emplace<Planning::PathTargetCommand>();
    }

    cmd_text_ << "face(" << pt << ")" << endl;

    auto& command = std::get<Planning::PathTargetCommand>(intent().motion_command);
    command.angle_override = Planning::TargetFacePoint{pt};
}

#pragma mark Motion

bool OurRobot::charged() const {
    return radio_status().kicker == RobotStatus::KickerState::kCharged && status_is_fresh();
}

/*
 * If the ball was recently sensed, then we believe the robot still has it. This
 * avoids noisiness in the ball sensor.
 */
bool OurRobot::has_ball() const {
    if ((RJ::now() - last_ball_sense_) < lost_ball_duration_) {
        return true;
    }
    return OurRobot::has_ball_raw();
}

bool OurRobot::has_ball_raw() const { return radio_status().has_ball && status_is_fresh(); }

bool OurRobot::ball_sense_works() const {
    // TODO(Kyle): Get error code from robot.
    return status_is_fresh();
}

bool OurRobot::kicker_works() const {
    return radio_status().kicker != RobotStatus::KickerState::kFailed && status_is_fresh();
}

bool OurRobot::chipper_available() const { return kicker_works() && *status()->chipper_enabled; }

bool OurRobot::kicker_available() const { return kicker_works() && *status()->kicker_enabled; }

bool OurRobot::dribbler_available() const {
    return *status()->dribbler_enabled && radio_status().motors_healthy[4] && status_is_fresh();
}

bool OurRobot::driving_available(bool require_all) const {
    if (!status_is_fresh()) {
        return false;
    }

    int c = 0;
    for (int i = 0; i < 4; ++i) {
        if (radio_status().motors_healthy[i]) {
            ++c;
        }
    }

    return require_all ? (c == 4) : (c == 3);
}

double OurRobot::kicker_voltage() const {
    if (!status_is_fresh()) {
        return 0;
    }

    return radio_status().kicker_voltage;
}

RobotStatus::HardwareVersion OurRobot::hardware_version() const {
    if (status_is_fresh()) {
        return radio_status().version;
    }
    return RobotStatus::HardwareVersion::kUnknown;
}

bool OurRobot::status_is_fresh(RJ::Seconds age) const {
    return (RJ::now() - radio_status().timestamp) < age;
}

RJ::Timestamp OurRobot::last_kick_time() const { return RJ::timestamp(last_kick_time_); }

void OurRobot::radio_rx_updated() {
    if (radio_status().kicker == RobotStatus::KickerState::kCharging &&
        last_kicker_status_ == RobotStatus::KickerState::kCharged) {
        last_kick_time_ = RJ::now();
    }
    last_kicker_status_ = radio_status().kicker;
}

double OurRobot::distance_to_chip_landing(int chip_power) {
    return max(0., min(190., *(config()->chipper.calibration_slope) * chip_power +
                                 *(config()->chipper.calibration_offset)));
}

uint8_t OurRobot::chip_power_for_distance(double distance) {
    double b = *(config()->chipper.calibration_offset) / 2.;
    if (distance < b) {
        return 0;
    }
    if (distance > distance_to_chip_landing(255)) {
        return 255;
    }
    return 0.5 * distance + b;
}

void OurRobot::set_pid(double p, double i, double d) {
    config()->translation.p->set_value_string(QString(std::to_string(p).c_str()));
    config()->translation.i->set_value_string(QString(std::to_string(i).c_str()));
    config()->translation.d->set_value_string(QString(std::to_string(d).c_str()));
}

void OurRobot::set_joystick_controlled(bool joystick_controlled) {
    context_->is_joystick_controlled[shell()] = joystick_controlled;
}

bool OurRobot::is_joystick_controlled() const { return context_->is_joystick_controlled[shell()]; }
