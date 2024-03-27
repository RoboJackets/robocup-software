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
 	latest_state_ = update_state();
    return state_to_task(intent);
};

SmartIdle::State SmartIdle::update_state() {
	switch(latest_state_) {
		case IDLING: {
            bool travel_upwards = false;
			double y_pos = last_world_state_->ball.position.y();
			if (y_pos - field_dimensions_.their_goal_loc().y() > 0) {
                travel_upwards = true;
            }
            double our_y_pos = last_world_state_->get_robot(true, this->robot_id_).pose.position().y();
			if ((!travel_upwards && y_pos - our_y_pos > 1) || (travel_upwards && y_pos - our_y_pos < 1)) {
				return GET_AWAY;
			}
			break;
		}
		case GET_AWAY: {
            bool travel_upwards = false;
			double y_pos = last_world_state_->ball.position.y();
			if (y_pos - field_dimensions_.their_goal_loc().y() > 0) {
                travel_upwards = true;
            }
            double our_y_pos = last_world_state_->get_robot(true, this->robot_id_).pose.position().y();
			if ((travel_upwards && y_pos - our_y_pos >= 1) || (!travel_upwards && y_pos - our_y_pos <= 1)) {
				return IDLING;
			}
			break;
		}
	}
	return latest_state_;
}


std::optional<RobotIntent> SmartIdle::state_to_task(RobotIntent intent) {
	switch(latest_state_) {
		case IDLING: {
			intent.motion_command = planning::MotionCommand{"escape_obstacles"};
	 		return intent;
		} case GET_AWAY: {
			bool travel_upwards = false;
			double y_pos = last_world_state_->ball.position.y();
			if (y_pos - field_dimensions_.their_goal_loc().y() > 0) {
                travel_upwards = true;
            }

            double offset = 1.5;
            if (travel_upwards) {
            	offset = -offset;
            }

            rj_geometry::Point target_pt{last_world_state_->get_robot(true, this->robot_id_).pose.position().x(), y_pos + offset};
            rj_geometry::Point target_vel{0.0, 0.0};

            planning::PathTargetFaceOption face_option{planning::FaceBall{}};

            planning::LinearMotionInstant goal{target_pt, target_vel};
            intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, false};

            return intent;
            break;
		}
	}
}

}  // namespace strategy
