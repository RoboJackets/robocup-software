#include "penalty_non_kicker.hpp"

#include <rclcpp/rclcpp.hpp>

#include "position.hpp"

namespace strategy {

PenaltyNonKicker::PenaltyNonKicker(int r_id) : Position{r_id, "PenaltyNonKicker"} {}

PenaltyNonKicker::PenaltyNonKicker(const Position& other) : Position{other} {}

std::string PenaltyNonKicker::get_current_state() { return "PenaltyNonKicker"; }

/**
 * @brief Does nothing; this position is a special case
 */
void PenaltyNonKicker::derived_acknowledge_pass() {}
/**
 * @brief Does nothing; this position is a special case
 */
void PenaltyNonKicker::derived_pass_ball(){

};
/**
 * @brief Does nothing; this position is a special case
 */
void PenaltyNonKicker::derived_acknowledge_ball_in_transit() {}

std::optional<RobotIntent> PenaltyNonKicker::derived_get_task(RobotIntent intent) {
    // latest_state_ = update_state();
    return state_to_task(intent);
};

// SmartIdle::State SmartIdle::update_state() {
//     switch (latest_state_) {
//         case IDLING: {
//         	bool travel_upwards = false;
//         	double y_pos = last_world_state_->ball.position.y();
//             if (y_pos - field_dimensions_.their_goal_loc().y() > 0) {
//                 travel_upwards = true;
//             }
//             if ((!travel_upwards && y_pos - last_world_state_->get_robot(true,
//             this->robot_id_).pose.position().y() > 0) || (travel_upwards && y_pos -
//             last_world_state_->get_robot(true, this->robot_id_).pose.position().y() < 0))  {
//                 return GET_AWAY;
//             }
//             break;
//         }
//         case GET_AWAY: {
//             if ((travel_upwards && y_pos - last_world_state_->get_robot(true,
//             this->robot_id_).pose.position().y() > 0) || (!travel_upwards && y_pos -
//             last_world_state_->get_robot(true, this->robot_id_).pose.position().y() < 0))  {
//               {
//                 return IDLING;
//             }
//             break;
//         }
//     }
//     return latest_state_;
// }

std::optional<RobotIntent> PenaltyNonKicker::state_to_task(RobotIntent intent) {
    double y_pos = last_world_state_->ball.position.y();
    double offset = 1.5;
    if (current_play_state_.is_our_restart()) {
        offset = -offset;
    }

    rj_geometry::Point target_pt{
        last_world_state_->get_robot(true, this->robot_id_).pose.position().x(), y_pos + offset};
    rj_geometry::Point target_vel{0.0, 0.0};

    planning::PathTargetFaceOption face_option{planning::FaceBall{}};

    planning::LinearMotionInstant goal{target_pt, target_vel};
    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, false};

    return intent;
}

}  // namespace strategy
