#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {

    // thread-safe getter
    WorldState* world_state = this->world_state();

    // Get Ball Location
    rj_geometry::Point ball_location{world_state->ball.position};

    // Create Waller
    Waller* waller = new Waller();
    return waller->get_task(intent, ball_location);
}

}  // namespace strategy
