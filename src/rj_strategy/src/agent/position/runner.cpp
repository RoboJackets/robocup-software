#include "rj_strategy/agent/position/runner.hpp"

#include <rj_common/planning/instant.hpp>
#include <rj_constants/constants.hpp>

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(const Position& other) : Position{other} {
    position_name_ = "Runner";
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    if (running_path_.empty()) {
        initialize_running_path();
    }

    State new_state = next_state();
    
    if (current_state_ != new_state) {
        SPDLOG_INFO("Robot {}: now {}", robot_id_, state_to_name(current_state_));
    }
    
    current_state_ = new_state;

    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return std::string{"Runner_"} + std::string{state_to_name(current_state_)};
}

Runner::State Runner::next_state() {
    if (has_reached_target()) {
        switch (current_state_) {
            case RUNNING_TO_CORNER_1:
                return RUNNING_TO_CORNER_2;
            case RUNNING_TO_CORNER_2:
                return RUNNING_TO_CORNER_3;
            case RUNNING_TO_CORNER_3:
                return RUNNING_TO_CORNER_4;
            case RUNNING_TO_CORNER_4:
                return RUNNING_TO_CORNER_1;
        }
    }
    
    return current_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    if (running_path_.empty()) {
        auto current_pos = last_world_state_->get_robot(true, robot_id_).pose.position();
        planning::LinearMotionInstant stay_target{current_pos};
        intent.motion_command = planning::MotionCommand{"path_target", stay_target};
        return intent;
    }

    rj_geometry::Point target = get_current_target();
    
    planning::LinearMotionInstant motion_target{target};
    intent.motion_command = planning::MotionCommand{"path_target", motion_target};
    
    return intent;
}

void Runner::initialize_running_path() {
    double field_width = field_dimensions_.width();
    double field_length = field_dimensions_.length();
    
    double rect_width = field_width * kRunningRectWidthRatio;
    double rect_length = field_length * kRunningRectLengthRatio;
    
    double half_width = rect_width / 2.0;
    double half_length = rect_length / 2.0;
    double center_x = 0.0;
    double center_y = field_length / 2.0;
    
    running_path_.clear();
    running_path_.push_back(rj_geometry::Point{center_x - half_width, center_y - half_length});
    running_path_.push_back(rj_geometry::Point{center_x + half_width, center_y - half_length});
    running_path_.push_back(rj_geometry::Point{center_x + half_width, center_y + half_length});
    running_path_.push_back(rj_geometry::Point{center_x - half_width, center_y + half_length});
    
    SPDLOG_INFO("Runner {}: Initialized running path with {} corners", robot_id_, running_path_.size());
}

bool Runner::has_reached_target() const {
    if (running_path_.empty()) {
        return false;
    }
    
    auto current_pos = last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point target = get_current_target();
    
    double distance = current_pos.dist_to(target);
    return distance < kReachedThreshold;
}

rj_geometry::Point Runner::get_current_target() const {
    if (running_path_.empty()) {
        return rj_geometry::Point{0.0, field_dimensions_.length() / 2.0};
    }
    
    int corner_index = 0;
    switch (current_state_) {
        case RUNNING_TO_CORNER_1:
            corner_index = 0;
            break;
        case RUNNING_TO_CORNER_2:
            corner_index = 1;
            break;
        case RUNNING_TO_CORNER_3:
            corner_index = 2;
            break;
        case RUNNING_TO_CORNER_4:
            corner_index = 3;
            break;
    }
    
    return running_path_[corner_index];
}

void Runner::derived_acknowledge_pass() {
    // pass
}

void Runner::derived_pass_ball() {
    // pass
}

void Runner::derived_acknowledge_ball_in_transit() {
    // pass
}

}