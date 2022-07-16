#pragma once

#include <functional>
#include <memory>
#include <thread>
#include <vector>

// ros2 action includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// rj includes
#include <rclcpp/duration.hpp>

#include <planning/planner/motion_command.hpp>
#include <radio/robot_status.hpp>
#include <rj_common/utils.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/ball_placement.hpp>
#include <rj_msgs/msg/is_done.hpp>
#include <rj_msgs/msg/robot_intent.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <world_state.hpp>

#include "planning/instant.hpp"
#include "planning/planner/plan_request.hpp"
#include "planning/planner_node.hpp"
#include "planning/trajectory.hpp"
#include "rj_geometry/point.hpp"
/* #include "robot_intent.hpp" */

namespace server {
class BallPlacementServer : public rclcpp::Node {
public:
    enum BallPlacementState { INIT, MOVING_TO_SPOTS, PASSING, RETREAT, DONE };

    using BallPlacement = rj_msgs::action::BallPlacement;
    using GoalHandleBallPlacement = rclcpp_action::ServerGoalHandle<BallPlacement>;

    BallPlacementServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~BallPlacementServer() = default;

private:
    // TODO(Alex): docstrings
    rclcpp_action::Server<BallPlacement>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const BallPlacement::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleBallPlacement> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleBallPlacement> goal_handle);
    void execute(const std::shared_ptr<GoalHandleBallPlacement> goal_handle);

    // TODO: what is the ball placement tolerance, per rules?
    const double BALL_PLACEMENT_TOLERANCE_ = 0.1;  // m

    // to keep track of duplicate goal pt requests
    rj_geometry::Point curr_goal_pt_;
    bool was_given_goal_pt_ = false;

    BallPlacementState current_state_ = BallPlacementState::INIT;

    // TODO: check RobotStatus?

    // this should all be gameplay lmao
    rclcpp::Publisher<rj_msgs::msg::RobotIntent>::SharedPtr robot_intent_0_pub_;
    rclcpp::Publisher<rj_msgs::msg::RobotIntent>::SharedPtr robot_intent_1_pub_;

    rclcpp::Subscription<rj_msgs::msg::IsDone>::SharedPtr is_done_0_sub_;
    rclcpp::Subscription<rj_msgs::msg::IsDone>::SharedPtr is_done_1_sub_;
    bool last_is_done_0_ = false;
    bool last_is_done_1_ = false;

    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    WorldState last_world_state_;

    // to track desired start positions pre-kick
    rj_geometry::Point robot_0_start_pt_;
    rj_geometry::Point robot_1_start_pt_;

    // to ensure only kicks once
    bool has_kicked_ = false;

    // to track desired end positions post ball placement
    rj_geometry::Point robot_0_end_pt_;
    rj_geometry::Point robot_1_end_pt_;
    bool end_pts_sent_ = false;
};
}  // namespace server
