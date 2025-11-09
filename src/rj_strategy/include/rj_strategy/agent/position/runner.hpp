#pragma once
#include "rj_strategy/agent/position.hpp"

namespace strategy {

    // creates a new class Runner that inherits from Position
    class Runner : public Position {
        public:
            // basic Runner constructor
            Runner(int r_id);
            Runner(const Position& other);

            std::string get_current_state() override;

            void update(const WorldState* world_state);

        private:
            std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

            // enum for private state variables
            enum State {
                SIDE_1,
                SIDE_2,
                SIDE_3,
                SIDE_4
            };

            State current_state_;
            State next_state(State s);
    };
}