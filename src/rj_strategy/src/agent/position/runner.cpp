#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(const Position& other) : Position{other} { position_name_ = "Runner"; }

std::string Runner::get_current_state() { return "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    State new_state = update_state();
    current_state_ = new_state;
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    switch (current_state_) {
        case RUNNING: {
            if (check_is_done()) {
                current_vertex_index_ = (current_vertex_index_ + 1) % kNumVertices;
            }
            return RUNNING;
        }
    }
    return RUNNING;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point center = field_dimensions_.center_field_loc();

    rj_geometry::Point vertices[kNumVertices] = {
        rj_geometry::Point{center.x() + 2.0, center.y() + 1.5},
        rj_geometry::Point{center.x() - 2.0, center.y() + 1.5},
        rj_geometry::Point{center.x() - 2.0, center.y() - 1.5},
        rj_geometry::Point{center.x() + 2.0, center.y() - 1.5}};

    switch (current_state_) {
        case RUNNING: {
            rj_geometry::Point target_vertex = vertices[current_vertex_index_];
            planning::LinearMotionInstant target{target_vertex, rj_geometry::Point{0.0, 0.0}};
            intent.motion_command =
                planning::MotionCommand{"path_target", target, planning::FaceAngle{0}, true};
            return intent;
        }
    }

    return intent;
}

}  // namespace strategy
