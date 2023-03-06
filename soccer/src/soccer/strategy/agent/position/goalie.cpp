#include "goalie.hpp"

namespace strategy {

// TODO(Kevin): lock Goalie id to id given by the ref
Goalie::Goalie(int r_id) : Position(r_id) { position_name_ = "Goalie"; }

std::optional<RobotIntent> Goalie::derived_get_task(RobotIntent intent) {
    // TEMP

    // line kick (PTMC can't handle 0 vel, and Empty can't handle the chip
    // command)
    auto line_kick_cmd = planning::LineKickMotionCommand{rj_geometry::Point{0.0, 1.5}};
    intent.motion_command = line_kick_cmd;
    intent.motion_command_name = "line kick";

    // send chip on breakbeam command
    intent.shoot_mode = RobotIntent::ShootMode::CHIP;
    intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
    intent.kick_speed = 4.0;
    intent.is_active = true;

    return intent;

    // END TEMP

    /* latest_state_ = update_state(); */
    /* return state_to_task(intent); */
}

Goalie::State Goalie::update_state() {
    // if a shot is coming, override all and go block it
    WorldState* world_state = this->world_state();

    // if no ball found, stop and return to box immediately
    if (!world_state->ball.visible) {
        return BALL_NOT_FOUND;
    }

    // if a shot is coming, override all and go block it
    if (shot_on_goal_detected(world_state)) {
        return BLOCKING;
    }

    // if the ball is in the goalie box, clear it
    bool ball_is_slow = world_state->ball.velocity.mag() < 0.5;  // m/s

    rj_geometry::Point ball_pt = world_state->ball.position;
    // TODO(Kevin): account for field direction when field coords
    // added in
    bool ball_in_box = ball_pt.y() < 1.0 && fabs(ball_pt.x()) < 1.0;  // m
    if (ball_is_slow && ball_in_box) {
        return CLEARING;
    }

    // otherwise, default to idling
    return IDLING;
}

std::optional<RobotIntent> Goalie::state_to_task(RobotIntent intent) {
    if (latest_state_ == BLOCKING) {
        auto intercept_cmd = planning::InterceptMotionCommand{rj_geometry::Point{0.0, 0.1}};
        intent.motion_command = intercept_cmd;
        intent.motion_command_name = "intercept";
        return intent;
    } else if (latest_state_ == IDLING) {
        auto goalie_idle_cmd = planning::GoalieIdleMotionCommand{};
        intent.motion_command = goalie_idle_cmd;
        intent.motion_command_name = "goalie_idle";
        return intent;
    } else if (latest_state_ == CLEARING) {
        auto line_kick_cmd = planning::LineKickMotionCommand{rj_geometry::Point{0.0, 4.5}};
        intent.motion_command = line_kick_cmd;
        intent.motion_command_name = "line kick";

        // note: the way this is set up makes it impossible to
        // shoot on time without breakbeam
        // TODO(Kevin): make intent hold a manip msg instead? to be cleaner?
        intent.shoot_mode = RobotIntent::ShootMode::CHIP;
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        intent.kick_speed = 4.0;
        intent.is_active = true;

        return intent;
    } else if (latest_state_ == BALL_NOT_FOUND) {
        // TODO: make point dependent on team
        rj_geometry::Point target_pt{0, 0.5};
        rj_geometry::Point target_vel{0.0, 0.0};

        planning::PathTargetFaceOption face_option = planning::FaceTarget{};

        // ball not found
        bool ignore_ball = true;

        planning::LinearMotionInstant goal{target_pt, target_vel};
        intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, ignore_ball};
        intent.motion_command_name = "path_target";
        return intent;
    }

    // should be impossible to reach, but this is equivalent to
    // sending an EmptyMotionCommand
    return std::nullopt;
}

bool Goalie::shot_on_goal_detected(WorldState* world_state) {
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

    bool shot_on_target =
        std::abs(cross_x) < 0.5;  // TODO(Kevin): add field to world_state to avoid hardcoding this
    bool ball_is_fast = ball_vel.mag() > 1.0;
    return ball_is_fast && shot_on_target;
}

}  // namespace strategy
