#include "runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner"){}

Runner::Runner(const Position& other) : Position{other} { position_name_ = "Runner";}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    // SPDLOG_INFO("waller length (sus) {}, {}", walling_robots_.size(), robot_id_);
    current_state_ = next_state();
    // waller_id_ = get_waller_id();
    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}
planning::LinearMotionInstant vertices[4] =  {
    planning::LinearMotionInstant({-2,8},{0,0}), 
    planning::LinearMotionInstant({-2,3},{0,0}),
    planning::LinearMotionInstant({0,3},{0,0}),
    planning::LinearMotionInstant({0,8},{0,0})
    };
int current_vertex = 0;
Runner::State Runner::next_state() {
    State next_state = current_state_;
    // handle transitions between states
    WorldState* world_state = last_world_state_;
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
        

    switch (current_state_) {
        case TURNING:
            // go to next element in the vertices array
            current_vertex = (current_vertex+1)%4;
            next_state = RUNNING;
        case RUNNING:
            // transition to turning once distance has been covered
            if (check_is_done()) {
                next_state = TURNING;
            }
            
            
    }
    return next_state;
}

std::optional<RobotIntent> strategy::Runner::state_to_task(RobotIntent intent) {
    
    if (current_state_ == RUNNING) {
        //face target and move towards it;
        //planning::PathTargetFaceOption face_option{vertices[current_vertex]};
            
            intent.motion_command =
                planning::MotionCommand{"path_target", vertices[current_vertex],planning::FaceBall{}, true};
            return intent;
    } else if (current_state_ == TURNING) {
        //face target and do nothing
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
    } 

    return std::nullopt;
}
} 
// namespace strategy

