#include "rj_strategy/agent/position/runner.hpp"
#include <iostream>

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = next_state();
    return state_to_task(intent);
}

Runner::State Runner::next_state() {
    rj_geometry::Point robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();

    if (robot_position.near_point(rj_geometry::Point{-1, 3}, 0.2)) {
        return RIGHT;
    } else if (robot_position.near_point(rj_geometry::Point{-1, 6}, 0.2)) {
        return BOTTOM;
    } else if (robot_position.near_point(rj_geometry::Point{1, 6}, 0.2)) {
        return LEFT;
    } else if (robot_position.near_point(rj_geometry::Point{1, 3}, 0.2)) {
        return TOP;
    }
    return current_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case DEFAULT: {
            current_state_ = TOP;
        }

        case TOP: {
            planning::MotionCommand motion_command{};

            rj_geometry::Point target_dest{-1, 3};
            rj_geometry::Point target_vel{0, 0};
            planning::LinearMotionInstant target{target_dest, target_vel};

            motion_command.name = "path_target";
            motion_command.target = target;

            intent.motion_command = motion_command;

            return intent;
        }

        case RIGHT: {
            planning::MotionCommand motion_command{};

            rj_geometry::Point target_dest{-1, 6};
            rj_geometry::Point target_vel{0, 0};
            planning::LinearMotionInstant target{target_dest, target_vel};

            motion_command.name = "path_target";
            motion_command.target = target;

            intent.motion_command = motion_command;

            return intent;

        }

        case BOTTOM: {
            planning::MotionCommand motion_command{};

            rj_geometry::Point target_dest{1, 6};
            rj_geometry::Point target_vel{0, 0};
            planning::LinearMotionInstant target{target_dest, target_vel};

            motion_command.name = "path_target";
            motion_command.target = target;

            intent.motion_command = motion_command;

            return intent;

        }

        case LEFT: {
            planning::MotionCommand motion_command{};

            rj_geometry::Point target_dest{1, 3};
            rj_geometry::Point target_vel{0, 0};
            planning::LinearMotionInstant target{target_dest, target_vel};

            motion_command.name = "path_target";
            motion_command.target = target;

            intent.motion_command = motion_command;

            return intent;
        }

    }
    return intent;
}

std::string Runner::get_current_state() {
    return "bleh";
}

}