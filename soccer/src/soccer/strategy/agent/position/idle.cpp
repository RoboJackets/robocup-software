#include "idle.hpp"

#include <rclcpp/rclcpp.hpp>

#include "position.hpp"

namespace strategy {

Idle::Idle(int r_id) : Position{r_id, "Idle"} {}

Idle::Idle(const Position& other) : Position{other} {}

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
