#pragma once

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_utils/logging.hpp>

#include "agent_action_client/position/goalie.hpp"
#include "agent_action_client/position/position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"
#include "world_state.hpp"

namespace strategy {

/*
 * The AgentActionClient houses the ROS connections for an agent in our
 * strategy system, but delegates most of the strategy logic to the Position
 * class. The goal is for this class to handle the ROS minutia, and for Position
 * to implement the strategy. See position.hpp for more details.
 */
class AgentActionClient : public rclcpp::Node {
public:
    using RobotMove = rj_msgs::action::RobotMove;
    using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

    // TODO: make robot_id, spin up N ACs
    AgentActionClient();

private:
    // TODO: sub to worldstate, give callback access to Position
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;

    // TODO(Kevin): communication module pub/sub here (e.g. passing)

    // TODO(Kevin): sub to coach node, once merged
    // TODO(Kevin): move this folder to strategy/, once coach merged

    std::unique_ptr<Position> current_position_;

    // ROS ActionClient spec, for calls to planning ActionServer
    rclcpp_action::Client<RobotMove>::SharedPtr client_ptr_;
    void goal_response_callback(std::shared_future<GoalHandleRobotMove::SharedPtr> future);
    void feedback_callback(GoalHandleRobotMove::SharedPtr,
                           const std::shared_ptr<const RobotMove::Feedback> feedback);

    void result_callback(const GoalHandleRobotMove::WrappedResult& result);

    // TODO: doc
    /*
     * @brief send a goal to the planning ActionServer, based on the Position's get_task().
     */
    void send_goal();

    rclcpp::TimerBase::SharedPtr get_task_timer_;
    void get_task();
    rj_msgs::msg::RobotIntent latest_task_;

};  // class AgentActionClient

}  // namespace strategy
