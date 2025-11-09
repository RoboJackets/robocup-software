#include "rj_strategy/agent/position/runner.hpp"

#include "rj_geometry/point.hpp"
#include "rj_common/planning/instant.hpp"
#include "rj_common/planning/motion_command.hpp"
#include <rj_common/planning/instant.hpp>

namespace strategy {
    Runner::Runner(int r_id) : Position(r_id), current_state_(State::SIDE_1) {}
    Runner::Runner(const Position& other) : Position(other), current_state_(SIDE_1) {}

    std::string Runner::get_current_state() {
        switch (current_state_) {
            case SIDE_1:
                return "SIDE_1";
            case SIDE_2:
                return "SIDE_2";
            case SIDE_3:
                return "SIDE_3";
            case SIDE_4:
                return "SIDE_4";
        }
        return "UNKNOWN";
    }
    
    Runner::State Runner::next_state(State s) {
        switch(s) {
            // In case SIDE_1, return SIDE_2
            // In case SIDE_2, return SIDE_3
            // etc
            case State::SIDE_1:
                return State::SIDE_2;
            case State::SIDE_2:
                return State::SIDE_3;
            case State::SIDE_3:
                return State::SIDE_4;
            case State::SIDE_4:
                return State::SIDE_1;
        }

        // If none of the above cases are met, return SIDE_1
        return State::SIDE_1;
    }
    
    std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
        // Get current robot position
        auto my_robot = last_world_state_->get_robot(true, robot_id_);
        auto current_pos = my_robot.pose.position();

        // Define a target point (example: move along x-axis)
        rj_geometry::Point target{1.0, current_pos.y()};

        // Create LinearMotionInstant from current to target
        planning::LinearMotionInstant motion{current_pos, target};

        // Assign to motion_command
        intent.motion_command = planning::MotionCommand{"path_target", motion};

        return intent;
    }

    // update method
    void Runner::update(const WorldState* world_state) {
        
        //sets my_robot to rj team robot with a given ID
        auto my_robot = world_state->get_robot(true, robot_id_);
        auto current_position = my_robot.pose.position();

        rj_geometry::Point target{3.0, 0.0};

        planning::MotionCommand motion_cmd{"path_target", planning::LinearMotionInstant(current_position, target)};
    }
}