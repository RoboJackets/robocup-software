#include "rj_strategy/agent/position/runner.hpp"

#include "rj_geometry/point.hpp"
#include "rj_common/planning/instant.hpp"
#include "rj_common/planning/motion_command.hpp"

namespace strategy {
    Runner::Runner(int r_id) : Position(r_id), current_state_(State::SIDE_1) {}

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

    // update method
    void Runner::update(const WorldState* world_state) {
        
        //sets my_robot to rj team robot with a given ID
        auto my_robot = world_state->get_robot(true, robot_id_);
        auto current_position = my_robot.pose.position();

        rj_geometry::Point target{3.0, 0.0};

        planning::MotionCommand motion_cmd{"path_target", planning::LinearMotionInstant(current_position, target)};
    }
}