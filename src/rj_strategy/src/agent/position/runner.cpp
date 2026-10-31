#include "rj_strategy/agent/position/runner.hpp"


namespace strategy {
    Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

    Runner::Runner(Position&& other) : Position{std::move(other)} {
        position_name_ = "Runner";
    }

    std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
        State new_state = next_state();
        
        current_state_ = new_state;

        return state_to_task(intent);     
    }

    Runner::State Runner::next_state() {

        if(!check_is_done()) {return current_state_;}

        switch (current_state_) {

            case SIDE_ONE: {
                return SIDE_TWO;
            }

            case SIDE_TWO: {
                return SIDE_THREE;
            }

            case SIDE_THREE: {
                return SIDE_FOUR;
            }

            case SIDE_FOUR: {
                return SIDE_ONE;
            }
        }
        return State::SIDE_ONE;
    }

    std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
        
        
        
        return intent;
    }
}