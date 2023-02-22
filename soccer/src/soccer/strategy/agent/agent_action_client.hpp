#pragma once

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/coach_state.hpp>
#include <rj_msgs/msg/position_ack.hpp>
#include <rj_msgs/msg/position_assignment.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_utils/logging.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"
#include "strategy/agent/position/defense.hpp"
#include "strategy/agent/position/goalie.hpp"
#include "strategy/agent/position/offense.hpp"
#include "strategy/agent/position/position.hpp"
#include "world_state.hpp"

// Communication
#include "communication/communication.hpp"
#include "rj_msgs/msg/acknowledge.hpp"
#include "rj_msgs/msg/agent_request.hpp"
#include "rj_msgs/srv/agent_communication.hpp"

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
    rclcpp::Subscription<rj_msgs::msg::PositionAssignment>::SharedPtr positions_sub_;
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

    /**
     * @brief send a goal to the planning ActionServer, based on the Position's get_task().
     */
    void send_new_goal();

    /**
     * @brief calls and executes current_positions_'s current desired task
     */
    void get_task();
    rclcpp::TimerBase::SharedPtr get_task_timer_;

    /*
     * Updates the current position based on the robot ID and the given Position message.
     */
    void update_position(const rj_msgs::msg::PositionAssignment::SharedPtr& msg);
    // note that this is our RobotIntent struct (robot_intent.hpp), not a
    // pre-generated ROS msg type
    RobotIntent last_task_;

    // Robot Communication //
    /**
     * @brief the callback that handles receiving and dealing with received agent communication
     *
     * @param request the robot communication request from the other robot
     * @param response the communication to return to the other robot
     */
    void receive_communication_callback(
        const std::shared_ptr<rj_msgs::srv::AgentCommunication::Request>& request,
        const std::shared_ptr<rj_msgs::srv::AgentCommunication::Response>& response);

    /**
     * @brief the callback that handles the response from any send transmissions to other agents.
     *
     * @param response the contents of the response from the other agent.
     * @param robot_id the robot id of the robot this response is from
     */
    void receive_response_callback(
        const std::shared_future<rj_msgs::srv::AgentCommunication::Response::SharedPtr>& response,
        u_int8_t robot_id);

    // server for receiving instructions from other agents
    rclcpp::Service<rj_msgs::srv::AgentCommunication>::SharedPtr robot_communication_srv_;

    // clients for receiving instructions from the other agents
    std::array<rclcpp::Client<rj_msgs::srv::AgentCommunication>::SharedPtr, kNumShells>
        robot_communication_cli_;

    rclcpp::TimerBase::SharedPtr get_communication_timer_;
    /**
     * @brief Get the communication object (request / response) from the current position.
     *
     */
    void get_communication();
    communication::AgentRequest last_communication_;
    std::vector<communication::AgentPosResponseWrapper> buffered_responses_;

    // timeout check
    /**
     * @brief Checks that less than timeout_duration_ has passed for each of the buffered responses.
     * If a response has been buffered for longer than the duration then the response (in its
     * current state) is sent to the position.
     *
     */
    void check_communication_timeout();
    const RJ::Seconds timeout_duration_ = RJ::Seconds(1);
    // End Robot Communication //

    // const because should never be changed, but initializer list will allow
    // us to set this once initially
    const int robot_id_;

    /*
     * @return thread-safe ptr to most recent world_state
     */
    [[nodiscard]] WorldState* world_state();
    WorldState last_world_state_;
    mutable std::mutex world_state_mutex_;

};  // class AgentActionClient

}  // namespace strategy
