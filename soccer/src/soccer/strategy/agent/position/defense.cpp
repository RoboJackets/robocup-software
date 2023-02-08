#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {
    WorldState* world_state = this->world_state();
    //Blocker blocker{};
    return intent;
}

}  // namespace strategy
