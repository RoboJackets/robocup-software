#include "rj_strategy/agent/position.hpp"

namespace strategy {
    class Runner : public Position {
        public:
            Runner(int r_id);
            ~Runner() override = default;
            Runner(const Position& other);

            std::string get_current_state() override;

        private:
            /**
             * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
             */
            std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

            enum State {
                DEFAULT,           // Decide what to do
                FORWARD,     // moving forward towards opponent's goal
                LEFT,           // moving left along opponent's end line
                BACKWARD // moving back towards its own goal,
                RIGHT // moving right along its own end line
            };

            /**
             * @return what the state should be right now. called on each get_task tick
             */
            State next_state();

            /**
             * @return the task to execute. called on each get_task tick AFTER next_state()
             */
            std::optional<RobotIntent> state_to_task(RobotIntent intent);
        
    }
}