#pragma once

#include <functional>
#include <memory>
#include <thread>

// ros2 action includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// rj includes
#include <rj_common/utils.hpp>
#include <rj_msgs/action/move.hpp>

namespace server {
    class MoveActionServer : public rclcpp::Node {
        public:
            MoveActionServer();
        private:
            rclcpp_action::Server<Move>::SharedPtr action_server_;
            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Move::Goal> goal);
            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle);
            void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);
            void execute(const std::shared_ptr<GoalHandleMove> goal_handle);

    }
} // namespace server
