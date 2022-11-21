#pragma once

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_msgs/msg/coach_state.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_msgs/msg/position.hpp>
#include <rj_utils/logging.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"
#include "strategy/agent/position/defense.hpp"
#include "strategy/agent/position/goalie.hpp"
#include "strategy/agent/position/offense.hpp"
#include "strategy/agent/position/position.hpp"
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

    AgentActionClient();
    AgentActionClient(int r_id);
    ~AgentActionClient() = default;

private:
    // ROS pub/subs
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::CoachState>::SharedPtr coach_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::Position>::SharedPtr positions_sub_;
    // TODO(Kevin): communication module pub/sub here (e.g. passing)

    // callbacks for subs
    void world_state_callback(const rj_msgs::msg::WorldState::SharedPtr& msg);
    void coach_state_callback(const rj_msgs::msg::CoachState::SharedPtr& msg);

    std::unique_ptr<Position> current_position_;

    // ROS ActionClient spec, for calls to planning ActionServer
    rclcpp_action::Client<RobotMove>::SharedPtr client_ptr_;
    void goal_response_callback(std::shared_future<GoalHandleRobotMove::SharedPtr> future);
    void feedback_callback(GoalHandleRobotMove::SharedPtr,
                           const std::shared_ptr<const RobotMove::Feedback> feedback);

    void result_callback(const GoalHandleRobotMove::WrappedResult& result);

    /*
     * @brief send a goal to the planning ActionServer, based on the Position's get_task().
     */
    void send_new_goal();

    /*
     * @brief calls and executes current_position_'s current desired task
     */
    void get_task();
    rclcpp::TimerBase::SharedPtr get_task_timer_;
    void get_task();
    rj_msgs::msg::RobotIntent last_task_;

    // const because should never be changed, but initializer list will allow
    // us to set this once initially
    const int robot_id_;

};  // class AgentActionClient

}  // namespace strategy
