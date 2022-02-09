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
#include <rj_common/utils.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/action/move.hpp>
#include <rj_msgs/msg/trajectory.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <world_state.hpp>

namespace server {
using RobotIntent = rj_msgs::msg::RobotIntent;
class MoveActionServer : public rclcpp::Node {
public:
    using Move = rj_msgs::action::Move;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

    MoveActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MoveActionServer() = default;

private:
    mutable std::mutex mutex_;
    std::array<rj_geometry::Point, kNumShells> target_pivots;
    std::array<rj_geometry::Point, kNumShells> target_positions;
    std::vector<std::shared_ptr<rclcpp::Publisher<RobotIntent>>> intent_pubs_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;

    [[nodiscard]] const WorldState* world_state() const {
        auto lock = std::lock_guard(mutex_);
        return &last_world_state_;
    }

    WorldState last_world_state_;
    std::vector<planning::Trajectory> robot_trajectories_;
    std::vector<RobotState> robot_desired_states_;
    std::vector<bool> test_desired_states_;
    std::vector<bool> test_accept_goal_;
    RobotState desired_state_;
    rclcpp_action::Server<Move>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const Move::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle);
    void execute(const std::shared_ptr<GoalHandleMove> goal_handle);
};
}  // namespace server
#endif
