#include "rj_strategy/agent/position/runner.hpp";

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

Runner::Runner(const Position& other) : Position{other} {
    position_name_ = "Runner";
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}
std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}

Runner::State Runner::update_state() {
    State next_state = current_state_;
    WorldState* world_state = last_world_state_;
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();

    switch (current_state_) {
        case RIGHT_SIDE:
            if (robot_position.x() < -5 / 2 &&
                robot_position.y() < 5 / 2) {
                next_state = TOP_SIDE;
            }
            break;
        case TOP_SIDE:
            if (robot_position.x() > 5 / 2 &&
                robot_position.y() < 5 / 2) {
                next_state = LEFT_SIDE;
            }
            break;
        case LEFT_SIDE:
            if (robot_position.x() > 5 / 2 &&
                robot_position.y() > 5 + 5 / 2) {
                next_state = BOTTOM_SIDE;
            }
            break;
        case BOTTOM_SIDE:
            if (robot_position.x() < -5 / 2 &&
                robot_position.y() > 5 + 5 / 2) {
                next_state = RIGHT_SIDE;
            }
            break;
    }

    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
            WorldState* world_state = last_world_state_;
            auto robot_position = world_state->get_robot(true, robot_id_).pose.position();


    if (current_state_ == RIGHT_SIDE) {
        auto target_point =  rj_geometry::Point(-5 / 2 - 0.1,
                                       5 / 2 - 0.1);
        planning::LinearMotionInstant target(target_point);
        auto move_to_target_cmd =
            planning::MotionCommand{"path_target", target};
        intent.motion_command = move_to_target_cmd;
        return intent;
    } else if (current_state_ == TOP_SIDE) {
        auto target_point =  rj_geometry::Point(5 / 2 + 0.1,
                                       5 / 2 - 0.1);
        planning::LinearMotionInstant target(target_point);
        auto move_to_target_cmd =
            planning::MotionCommand{"path_target", target};
        intent.motion_command = move_to_target_cmd;
        return intent;
    } else if (current_state_ == LEFT_SIDE) {
        auto target_point = rj_geometry::Point(5 / 2 + 0.1,
                                       (5 + 5 / 2  + 0.1));
       planning::LinearMotionInstant target(target_point);
        auto move_to_target_cmd =
            planning::MotionCommand{"path_target", target};
        intent.motion_command = move_to_target_cmd;
        return intent;
    } else {
        auto target_point = rj_geometry::Point((-5 / 2 - 0.1),
                                        (5 + 5 / 2  + 0.1));
        planning::LinearMotionInstant target(target_point);
        auto move_to_target_cmd =
            planning::MotionCommand{"path_target", target};
        intent.motion_command = move_to_target_cmd;
        return intent;
    }


}

void Runner::die() {

}

void Runner::revive() {
    current_state_ = RIGHT_SIDE;
}
}