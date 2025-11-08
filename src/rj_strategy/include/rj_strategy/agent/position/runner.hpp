#pragma once
#include "position.hpp"

namespace strategy {

    // creates a new class Runner that inherits from Position
    class Runner : public Position {
        public:
            // basic Runner constructor
            Runner(int r_id);

        private:
            // enum for private state variables
            enum State {
                SIDE_1,
                SIDE_2,
                SIDE_3,
                SIDE_4
        };
    };
}