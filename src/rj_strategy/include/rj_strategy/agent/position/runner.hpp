#include "rj_strategy/agent/position.hpp"

namespace strategy {
    class Runner : public Position {
        public:
            Runner(int r_id);
            ~Runner() override = default;
            Runner(const Position& other);

    }
}