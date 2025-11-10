// ros specific imports
#include <rclcpp/rclcpp.hpp>

//position class that runner is inheriting
#include "rj_strategy/agent/position.hpp"

namespace strategy {
    // class Runner that inherits from Position
    class Runner : public Position {
        public:
            // "standard" constructor we will be using
            Runner(int runner_id);
            /* Class destructor*/
            // things you can do right before constructor is erased from memory
            ~Runner() = default;
            Runner(const Position& other);

            // overrides the get_current_state method from position 
            std::string get_current_state() override;

        private:
            // updates current state, based on update return state_to_task (kind of like a wrapper for state_to_task)
            std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

            // enum of all the different States we can have
            enum State {TOP_LEFT, TOP_RIGHT, BOTTOM_RIGHT, BOTTOM_LEFT};

            State current_state_ = State::TOP_LEFT;

            State next_state();

            // based the current state, what do we need to do
            std::optional<RobotIntent> state_to_task(RobotIntent intent);
    };
}