#include <rclcpp/rclcpp.hpp>
#include "rj_strategy/agent/position.hpp"

namespace strategy {
    class Runner : public Position {
        public:
            Runner(int runner_id); // "standard" constructor we will be using
            /* Class destructor*/
            ~Runner() = default;
            Runner(const Position& other);

            // function extended
            std::string get_current_state() override;

        private:
            std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
            enum State {TOP_LEFT, TOP_RIGHT, BOTTOM_RIGHT, BOTTOM_LEFT};

            State current_state_ = State::TOP_LEFT;

            State next_state();
            std::optional<RobotIntent> state_to_task(RobotIntent intent);
    };
}