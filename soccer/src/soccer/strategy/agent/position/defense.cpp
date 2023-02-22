#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {

    // thread-safe getter
    WorldState* world_state = this->world_state();

    // Create Waller
    Waller waller{0};
    return waller.get_task(intent, world_state);
}

}  // namespace strategy
