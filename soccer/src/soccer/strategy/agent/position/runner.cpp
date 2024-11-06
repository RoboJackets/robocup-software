#include "runner.hpp"

#include <rclcpp/rclcpp.hpp>

#include "position.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(const Position& other) : Position{other} {}

std::string Runner::get_current_state() { return "Runner"; }

/**
 * @brief Does nothing; this position is a special case
 */
void Runner::derived_acknowledge_pass() {}
/**
 * @brief Does nothing; this position is a special case
 */
void Runner::derived_pass_ball(){

};
/**
 * @brief Does nothing; this position is a special case
 */
void Runner::derived_acknowledge_ball_in_transit() {}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    // update current state 
    current_state_ = next_state(); 
    return state_to_task(intent); 
};

Runner::State Runner::next_state() {
	if(!check_is_done()) {
		return current_state_; // if ur not done yet, stay in same state 
	}
	
	// if check is done: 
	switch(current_state_){
		case TOP_LEFT:
			return TOP_RIGHT; 
		case TOP_RIGHT:
			return BOTTOM_RIGHT;
		case BOTTOM_RIGHT:
			return BOTTOM_LEFT;
		case BOTTOM_LEFT:
			return TOP_LEFT;
		default:
			return current_state_; 
	}
}
std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
	planning::LinearMotionInstant target;
	switch (current_state_) {
		case TOP_LEFT:
			target = planning::LinearMotionInstant{rj_geometry::Point{2.0,2.5}};
			break;
		case TOP_RIGHT:
			target = planning::LinearMotionInstant{rj_geometry::Point{-2.0,2.5}};
			break;
		case BOTTOM_RIGHT:
			target = planning::LinearMotionInstant{rj_geometry::Point{-2.0,6.5}};
			break;
		case BOTTOM_LEFT:
			target = planning::LinearMotionInstant{rj_geometry::Point{2.0,6.5}};
			break;
	}
	
	planning::MotionCommand command{"path_target", target, planning::FaceTarget{}};
	intent.motion_command = command; 
	return intent; 
}

}  // namespace strategy
