#include "runner.hpp"

#include <spdlog/spdlog.h>

namespace strategy {

Runner::Runner(int r_id) : Position(r_id) { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    State next_state = current_state_;
    WorldState* world_state = this->world_state();
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();

    switch(current_state_) {
        case RUNFIRST:
            if (check_is_done()) {
                next_state = RUNSECOND;
            }
            break;
        case RUNSECOND:
            if (check_is_done()) {
                next_state = RUNTHIRD;
            }
            break;
        case RUNTHIRD:
            if (check_is_done()) {
                next_state = RUNFOURTH;
            }
            break;
        case RUNFOURTH:
            if (check_is_done()) {
                next_state = RUNFIRST;
            }
            break;
    }

    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    if (current_state_ == RUNFIRST) {
        auto motion_instance = planning::LinearMotionInstant{rj_geometry::Point{3.0, 9.0}, rj_geometry::Point{0,0}};
        auto face_target = planning::FaceTarget{};
        auto moveFirstVertex = planning::MotionCommand{"path_target", motion_instance, face_target};
        intent.motion_command = moveFirstVertex;
    } else if (current_state_ == RUNSECOND) {
        auto motion_instance = planning::LinearMotionInstant{rj_geometry::Point{3.0, 0.0}, rj_geometry::Point{0,0}};
        auto face_target = planning::FaceTarget{};
        auto moveSecondVertex = planning::MotionCommand{"path_target", motion_instance, face_target};
        intent.motion_command = moveSecondVertex;
    } else if (current_state_ == RUNTHIRD) {
        auto motion_instance = planning::LinearMotionInstant{rj_geometry::Point{-3.0, 0.0}, rj_geometry::Point{0,0}};
        auto face_target = planning::FaceTarget{};
        auto moveThirdVertex = planning::MotionCommand{"path_target", motion_instance, face_target};
        intent.motion_command = moveThirdVertex;
    } else if (current_state_ == RUNFOURTH) {
        auto motion_instance = planning::LinearMotionInstant{rj_geometry::Point{-3.0, 9.0}, rj_geometry::Point{0,0}};
        auto face_target = planning::FaceTarget{};
        auto moveFourthVertex = planning::MotionCommand{"path_target", motion_instance, face_target};
        intent.motion_command = moveFourthVertex;
    }
    return intent;
}

}