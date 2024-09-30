#include "runner.hpp"

#include <spdlog/spdlog.h>

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

Runner::Runner(const Position& other) : Position{other} {}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

std::string Runner::get_current_state() { return "Runner"; }

bool inPosition(rj_geometry::Point current, rj_geometry::Point target, double tolerance) {
    return current.dist_to(target) < tolerance;
}

Runner::State Runner::update_state() {
    auto position = last_world_state_->get_robot(true, robot_id_).pose.position();

    if (latest_state_ == SIDE0 && inPosition(position, rj_geometry::Point(4, 4), 0.3)) {
        return SIDE1;
    } else if (latest_state_ == SIDE1 && inPosition(position, rj_geometry::Point(4, 8), 0.3)) {
        return SIDE2;
    } else if (latest_state_ == SIDE2 && inPosition(position, rj_geometry::Point(-4, 8), 0.3)) {
        return SIDE3;
    } else if (latest_state_ == SIDE3 && inPosition(position, rj_geometry::Point(-4, 4), 0.3)) {
        return SIDE0;
    }

    if (latest_state_ < SIDE0) {
        return SIDE0;
    }

    return latest_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    if (latest_state_ == SIDE0) {
        auto target = rj_geometry::Point(4, 4);
        auto mark_cmd = planning::MotionCommand{
            "path_target", planning::LinearMotionInstant{target}, planning::FaceBall{}, true};
        intent.motion_command = mark_cmd;
    } else if (latest_state_ == SIDE1) {
        auto target = rj_geometry::Point(4, 8);
        auto mark_cmd = planning::MotionCommand{
            "path_target", planning::LinearMotionInstant{target}, planning::FaceBall{}, true};
        intent.motion_command = mark_cmd;
    } else if (latest_state_ == SIDE2) {
        auto target = rj_geometry::Point(-4, 8);
        auto mark_cmd = planning::MotionCommand{
            "path_target", planning::LinearMotionInstant{target}, planning::FaceBall{}, true};
        intent.motion_command = mark_cmd;
    } else if (latest_state_ == SIDE3) {
        auto target = rj_geometry::Point(-4, 4);
        auto mark_cmd = planning::MotionCommand{
            "path_target", planning::LinearMotionInstant{target}, planning::FaceBall{}, true};
        intent.motion_command = mark_cmd;
    }
    return intent;

    // should be impossible to reach, but this is equivalent to
    // sending an empty MotionCommand
    return std::nullopt;
}

bool Runner::shot_on_goal_detected(WorldState* world_state) {
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;

    // find x-coord that the ball would cross on the goal line to figure out if
    // shot is on target ((0, 0) is our goal, +y points out of goal)
    //
    // assumes ball vel will remain constant
    // TODO(Kevin): account for acceleration?
    if (ball_vel.y() == 0) {
        return false;
    }
    double time_to_cross = std::abs(ball_pos.y() / ball_vel.y());
    double cross_x = ball_pos.x() + ball_vel.x() * time_to_cross;

    bool shot_on_target = std::abs(cross_x) < this->field_dimensions_.goal_width() / 2.0;
    bool in_direction = ball_vel.y() < 0;

    bool ball_is_fast = ball_vel.mag() > 1.0;
    return ball_is_fast && shot_on_target && in_direction;
}

void Runner::derived_acknowledge_pass() { latest_state_ = FACING; }

void Runner::derived_pass_ball() { latest_state_ = PASSING; }

void Runner::derived_acknowledge_ball_in_transit() { latest_state_ = RECEIVING; }

rj_geometry::Point Runner::penalty_location() {
    // be dumb: center of baseline
    return this->field_dimensions_.our_goal_loc();
    // be smart
    // find robot on their team closest to ball
    // line up in line with them and the ball on the baseline
}

}  // namespace strategy
