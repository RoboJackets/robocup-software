#include "zoner.hpp"

namespace strategy {

Zoner::Zoner(int r_id) : Position(r_id, "Zoner") {}

Zoner::Zoner(const Position& other) : Position{other} {
    position_name_ = "Zoner";
}

std::optional<RobotIntent> Zoner::derived_get_task(RobotIntent intent) {
    // SPDLOG_INFO("waller length (sus) {}, {}", walling_robots_.size(), robot_id_);
    current_state_ = update_state();
    // waller_id_ = get_waller_id();
    return state_to_task(intent);
}


}  // namespace strategy
