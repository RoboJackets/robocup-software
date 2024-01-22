#include "runner.hpp"

#include <spdlog/spdlog.h>

namespace strategy {


Runner::Runner(int r_id) : Position(r_id) {
    position_name_ = "Runner";
    current_state_ = RUN_ONE;
}
std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    State next_state = current_state_;
    // handle transitions between current state
    WorldState* world_state = this->world_state();

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();

    if (check_is_done()) {
        if (current_state_ == RUN_ONE) {
            next_state = RUN_TWO;
        }
        else if (current_state_ == RUN_TWO) {
            next_state = RUN_THREE;
        }
        else if (current_state_ == RUN_THREE) {
            next_state = RUN_FOUR;
        }
        else if (current_state_ == RUN_FOUR) {
            next_state = RUN_ONE;
        }
    }

    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) { //calculate the point you're going to (sides of the field) and move there
        if (current_state_ == RUN_ONE) {
            auto motion_instance = planning::LinearMotionInstant{rj_geometry::Point{3.0, 9.0}, rj_geometry::Point{0,0}};
            auto face_target = planning::FaceTarget{};
            auto moveVertex = planning::MotionCommand{"path_target", motion_instance, face_target};
            intent.motion_command = moveVertex;
        }
        else if (current_state_ == RUN_TWO) {
            auto motion_instance = planning::LinearMotionInstant{rj_geometry::Point{3.0, 0.0}, rj_geometry::Point{0,0}};
            auto face_target = planning::FaceTarget{};
            auto moveVertex = planning::MotionCommand{"path_target", motion_instance, face_target};
            intent.motion_command = moveVertex;
        }
        else if (current_state_ == RUN_THREE) {
            auto motion_instance = planning::LinearMotionInstant{rj_geometry::Point{-3.0, 0.0}, rj_geometry::Point{0,0}};
            auto face_target = planning::FaceTarget{};
            auto moveVertex = planning::MotionCommand{"path_target", motion_instance, face_target};
            intent.motion_command = moveVertex;
        }
        else if (current_state_ == RUN_FOUR) {
            auto motion_instance = planning::LinearMotionInstant{rj_geometry::Point{-3.0, 9.0}, rj_geometry::Point{0,0}};
            auto face_target = planning::FaceTarget{};
            auto moveVertex = planning::MotionCommand{"path_target", motion_instance, face_target};
            intent.motion_command = moveVertex;
        }
        return intent;
    }


}