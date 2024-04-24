#include "smartidling.hpp"

#include <rclcpp/rclcpp.hpp>

#include "position.hpp"

namespace strategy {

SmartIdle::SmartIdle(int r_id) : Position{r_id, "SmartIdle"} {}

SmartIdle::SmartIdle(const Position& other) : Position{other} {}

std::string SmartIdle::get_current_state() { return "SmartIdle"; }

/**
 * @brief Does nothing; this position is a special case
 */
void SmartIdle::derived_acknowledge_pass() {}
/**
 * @brief Does nothing; this position is a special case
 */
void SmartIdle::derived_pass_ball(){

};
/**
 * @brief Does nothing; this position is a special case
 */
void SmartIdle::derived_acknowledge_ball_in_transit() {}

std::optional<RobotIntent> SmartIdle::derived_get_task(RobotIntent intent) {
    intent.motion_command = planning::MotionCommand{"escape_obstacles"};
    return intent;
};

}  // namespace strategy
