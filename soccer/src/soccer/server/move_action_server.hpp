#ifndef MOVE_ACTION_SERVER_H
#define MOVE_ACTION_SERVER_H

#include <functional>
#include <memory>
#include <thread>
#include <vector>

// ros2 action includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// rj includes
#include <rj_common/utils.hpp>
#include <rj_msgs/action/move.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/trajectory.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <rj_constants/constants.hpp>

namespace server {
using RobotIntent = rj_msgs::msg::RobotIntent;
class MoveActionServer : public rclcpp::Node {
public:
    using Move = rj_msgs::action::Move;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

    MoveActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::vector<std::shared_ptr<rclcpp::Publisher<RobotIntent>>> intent_pubs_;
    rclcpp_action::Server<Move>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const Move::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);
    void execute(const std::shared_ptr<GoalHandleMove> goal_handle);
};
}  // namespace server
#endif
