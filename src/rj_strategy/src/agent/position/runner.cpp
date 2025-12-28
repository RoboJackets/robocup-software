#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {
    Runner::Runner(int r_id) : Position{r_id, "Runner"}{}

    Runner::Runner(const Position& other) : Position{other} {
    position_name_ = "Runner";
}
}