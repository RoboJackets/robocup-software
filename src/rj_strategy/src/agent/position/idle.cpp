#include "rj_strategy/agent/position/idle.hpp"

namespace strategy {

Idle::Idle(int r_id) : Position{r_id, "Idle"} {}

Idle::Idle(Position&& other) : Position{std::move(other)} {}

std::string Idle::get_current_state() { return "Idle"; }

/**
 * @brief Does nothing; this position is a special case
 */
void Idle::derived_acknowledge_pass() {}
/**
 * @brief Does nothing; this position is a special case
 */
void Idle::derived_pass_ball(){

};
/**
 * @brief Does nothing; this position is a special case
 */
void Idle::derived_acknowledge_ball_in_transit() {}

std::optional<RobotIntent> Idle::derived_get_task(RobotIntent intent) { return intent; };

}  // namespace strategy
