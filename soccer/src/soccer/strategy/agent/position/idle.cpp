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

std::optional<RobotIntent> Idle::derived_get_task(RobotIntent intent) { 
    double rulesCompliance = 0.6;
    rj_geometry::Point ball_pos = last_world_state_->ball.position;
    rj_geometry::Point goal_pos = field_dimensions_.our_goal_loc();

    rj_geometry::Point shot_dir = (goal_pos - ball_pos).normalized();
    rj_geometry::Point shot_dot = ball_pos + shot_dir * rulesCompliance;

    auto pivot_cmd =
        planning::MotionCommand{"path_target", planning::LinearMotionInstant{shot_dot},
                                planning::FaceBall{}, false};

    intent.motion_command = pivot_cmd;

    return intent;
};

}  // namespace strategy
