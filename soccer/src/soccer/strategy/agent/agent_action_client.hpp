#pragma once

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_msgs/msg/coach_state.hpp>
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
#include "rj_msgs/srv/agent_communication.hpp"
#include "rj_msgs/msg/agent_request.hpp"
#include "rj_msgs/msg/agent_request.hpp"
#include "rj_msgs/msg/agent_to_pos_comm_response.hpp"
#include "rj_msgs/msg/pos_to_agent_comm_request.hpp"
#include "rj_msgs/msg/acknowledge.hpp"

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

    // server for receiving instructions from other agents
    rclcpp::Service<rj_msgs::srv::AgentCommunication>::SharedPtr robot_communication_srv_;

    // clients for receiving instructions from the other agents
    rclcpp::Client<rj_msgs::srv::AgentCommunication>::SharedPtr robot_communication_cli_[kNumShells];

    WorldState last_world_state_;
    mutable std::mutex world_state_mutex_;
    /*
    * @return thread-safe ptr to most recent world_state
    */
    [[nodiscard]] WorldState* world_state();

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

    /**
     * @brief sends a robot communication to a specific robot
     * 
     * @param robot_id the robot to communicate with
     */
    void send_unicast(int robot_id, rj_msgs::msg::AgentRequest request);

    /**
     * @brief sends a robot communication to all robots
     * 
     */
    void send_broadcast(rj_msgs::msg::AgentRequest request);

    /**
     * @brief sends a robot communication to a specific group of robots
     * 
     * @param robot_ids the robot ids to send communication to
     */
    void send_multicast(std::vector<u_int8_t> robot_ids, rj_msgs::msg::AgentRequest agent_request);

    /**
     * @brief the callback that handles receiving and dealing with received agent communication
     * 
     * @param request the robot communication request from the other robot
     * @param response the communication to return to the other robot
     */
    void receive_communication_callback(const std::shared_ptr<rj_msgs::srv::AgentCommunication::Request>& request,
                                        std::shared_ptr<rj_msgs::srv::AgentCommunication::Response>& response);

    /**
     * @brief the callback that handles the response from any send transmissions to other agents.
     * 
     * @param response the contents of the response from the other agent.
     */
    void receive_response_callback(const std::shared_future<rj_msgs::srv::AgentCommunication::Response::SharedPtr>& response, int robot_id);

    // TODO(#1957): add back this if needed, or delete
    // cancel latest goal every time a new goal comes in, to avoid overloading memory with many
    // threads
    /* void cancel_last_goal(); */
    // after goal is asynchronously canceled, reset last_goal_handle_
    /* void cancel_goal_callback(rclcpp_action::Client<RobotMove>::CancelResponse::SharedPtr); */
    /* GoalHandleRobotMove::SharedPtr last_goal_handle_; */

    rclcpp::TimerBase::SharedPtr get_task_timer_;
    void get_task();
    rj_msgs::msg::RobotIntent last_task_;

    // Robot Communication
    rclcpp::TimerBase::SharedPtr get_communication_timer_;
    void get_communication();
    rj_msgs::msg::AgentRequest last_communication_;

    // const because should never be changed, but initializer list will allow
    // us to set this once initially
    const int robot_id_;

};  // class AgentActionClient

}  // namespace strategy
