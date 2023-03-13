#include "goalie.hpp"

namespace strategy {

// TODO(Kevin): lock Goalie id to id given by the ref
Goalie::Goalie(int r_id) : Position(r_id) { position_name_ = "Goalie"; }

std::optional<RobotIntent> Goalie::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
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

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    double distance_to_ball = robot_position.dist_to(ball_pt);
    if (latest_state_ == PASSING) {
        if (!ball_in_box) {
            return IDLING;
        }
    } else if (latest_state_ == RECEIVING) {
        if (distance_to_ball < 0.1) {
            return IDLING;
        }
    }

    // otherwise, default to idling
    return latest_state_;
}

std::optional<RobotIntent> Goalie::state_to_task(RobotIntent intent) {
    if (latest_state_ == IDLING) {
        auto goalie_idle_cmd = planning::GoalieIdleMotionCommand{};
        intent.motion_command = goalie_idle_cmd;
        intent.motion_command_name = "goalie_idle";
        return intent;
    } else if (latest_state_ == BLOCKING) {
        auto blocking_intercept_cmd =
            planning::InterceptMotionCommand{rj_geometry::Point{0.0, 0.1}};
        intent.motion_command = blocking_intercept_cmd;
        intent.motion_command_name = "intercept";
        return intent;
    } else if (latest_state_ == CLEARING) {
        auto clear_kick_cmd = planning::LineKickMotionCommand{rj_geometry::Point{0.0, 4.5}};
        intent.motion_command = clear_kick_cmd;
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
    } else if (latest_state_ == RECEIVING) {
        // intercept the bal
        rj_geometry::Point current_position =
            world_state()->get_robot(true, robot_id_).pose.position();
        auto receive_intercept_cmd = planning::InterceptMotionCommand{current_position};
        intent.motion_command = receive_intercept_cmd;
        intent.motion_command_name = fmt::format("robot {} goalie receive ball", robot_id_);
        return intent;
    } else if (latest_state_ == PASSING) {
        // attempt to pass the ball to the target robot
        rj_geometry::Point target_robot_pos =
            world_state()->get_robot(true, target_robot_id).pose.position();
        auto pass_kick_cmd = planning::LineKickMotionCommand{target_robot_pos};
        intent.motion_command = pass_kick_cmd;
        intent.motion_command_name =
            fmt::format("robot {} goalie pass to robot {}", robot_id_, target_robot_id);
        intent.shoot_mode = RobotIntent::ShootMode::KICK;
        // NOTE: Check we can actually use break beams
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        // TODO: Adjust the kick speed based on distance
        intent.kick_speed = 4.0;
        intent.is_active = true;
        return intent;
    }

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

communication::Acknowledge Goalie::acknowledge_pass(
    communication::IncomingPassRequest incoming_pass_request) {
    // Call to super
    communication::Acknowledge acknowledge_response =
        Position::acknowledge_pass(incoming_pass_request);
    // Update current state
    latest_state_ = FACING;
    // Return acknowledge response
    return acknowledge_response;
}

void Goalie::pass_ball(int robot_id) {
    // Call to super
    Position::pass_ball(robot_id);
    // Update current state
    latest_state_ = PASSING;
}

communication::Acknowledge Goalie::acknowledge_ball_in_transit(
    communication::BallInTransitRequest ball_in_transit_request) {
    // Call to super
    communication::Acknowledge acknowledge_response =
        Position::acknowledge_ball_in_transit(ball_in_transit_request);
    // Update current state
    latest_state_ = RECEIVING;
    // Return acknowledge response
    return acknowledge_response;
}

}  // namespace strategy
