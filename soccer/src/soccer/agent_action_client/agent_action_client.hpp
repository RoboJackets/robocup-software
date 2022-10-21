#pragma once

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_utils/logging.hpp>

#include "agent_action_client/position/position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

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

private:
    // TODO: sub to worldstate, give callback access to Position
    // TODO(Kevin): communication module pub/sub here (e.g. passing)

    // TODO(Kevin): sub to coach node, once merged
    // TODO(Kevin): move this folder to strategy/, once coach merged
    // TODO(Kevin): trip goal requests on coach, not on timer
    rclcpp::TimerBase::SharedPtr timer_;

    Position current_position_;

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

};  // class AgentActionClient

}  // namespace strategy
