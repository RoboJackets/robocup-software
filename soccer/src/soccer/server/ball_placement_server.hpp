#pragma once

#include <functional>
#include <memory>
#include <thread>
#include <vector>

// ros2 action includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// rj includes
#include <planning/planner/motion_command.hpp>
#include <radio/robot_status.hpp>
#include <rj_common/utils.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/ball_placement.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <world_state.hpp>

namespace server {
class BallPlacementServer : public rclcpp::Node {
public:
    using BallPlacement = rj_msgs::action::BallPlacement;
    using GoalHandleBallPlacement = rclcpp_action::ServerGoalHandle<BallPlacement>;

    BallPlacementServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~BallPlacementServer() = default;

private:
    rclcpp_action::Server<BallPlacement>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const BallPlacement::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleBallPlacement> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleBallPlacement> goal_handle);
    void execute(const std::shared_ptr<GoalHandleBallPlacement> goal_handle);

    // to keep track of duplicate goal pt requests
    rj_geometry::Point curr_goal_pt_;
    bool was_given_goal_pt_ = false;
};
}  // namespace server
