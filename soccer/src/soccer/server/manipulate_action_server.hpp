#ifndef MOVE_ACTION_SERVER_H
#define MOVE_ACTION_SERVER_H

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
#include <rj_msgs/action/manipulate.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <world_state.hpp>

namespace server {
using RobotIntent = rj_msgs::msg::RobotIntent;
class ManipulateActionServer : public rclcpp::Node {
public:
    using Manipulate = rj_msgs::action::Manipulate;
    using GoalHandleManipulate = rclcpp_action::ServerGoalHandle<Manipulate>;

    ManipulateActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ManipulateActionServer() = default;

private:
    struct kicker_info {
        bool kicker_charged;
        bool kicker_healthy;
    };

    std::vector<std::shared_ptr<rclcpp::Publisher<RobotIntent>>> intent_pubs_;
    std::vector<bool> kick_avl;
    std::vector<std::mutex> accept_goal;

    std::vector<kicker_info> robot_kicker_statuses_;
    rclcpp_action::Server<Manipulate>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const Manipulate::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleManipulate> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleManipulate> goal_handle);
    void execute(const std::shared_ptr<GoalHandleManipulate> goal_handle);
};
}  // namespace server
#endif
