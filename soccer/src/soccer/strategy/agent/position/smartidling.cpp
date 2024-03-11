#include "idle.hpp"

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
	rj_geometry::Point ball_position = last_world_state_->ball.position;
	planning::LinearMotionInstant target{ball_position};
 	planning::MotionCommand prep_command{"escape_obstacles", target, planning::FaceBall{}};

 	intent.motion_command = prep_command;

 	return intent; 
};

}  // namespace strategy