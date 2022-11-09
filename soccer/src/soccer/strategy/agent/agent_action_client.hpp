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
#include <rj_utils/logging.hpp>

#include "planning/planner/motion_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"
#include "robot_intent.hpp"
#include "rj_msgs/srv/AgentCommunication.srv"
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

    // server for receiving instructions from other agents
    rclcpp::Service<rj_msgs::srv::AgentCommunication>::SharedPtr robot_communication_server_;

    // clients for receiving instructions from the other agents
    rclcpp::Client<rj_msgs::srv::AgentCommunication>::SharedPtr robot_communication_clients_[kNumShells];

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

<<<<<<< HEAD
    /*
     * @brief calls and executes current_position_'s current desired task
     */
=======
    /**
     * @brief sends a robot communication to a specific robot
     * 
     * @param robot_id the robot to communicate with
     */
    void send_unicast(int&& robot_id);

    /**
     * @brief sends a robot communication to all robots
     * 
     */
    void send_broadcast();

    /**
     * @brief sends a robot communication to a specific group of robots
     * 
     * @param robot_ids the robot ids to send communication to
     */
    void send_multicast(int&& robot_ids[]);

    /**
     * @brief sends a robot communication and accepts only the first response
     * 
     * *NOTE* this is not the exact same as anycast in IP routing, but pretty similar
     * 
     */
    void send_anycast();

    void receive_communication_callback(const std::shared_ptr<rj_msgs::srv::AgentCommunication::Request> request,
                                        std::shared_ptr<rj_msgs::srv::AgentCommunication::Response> response);

    // TODO(#1957): add back this if needed, or delete
    // cancel latest goal every time a new goal comes in, to avoid overloading memory with many
    // threads
    /* void cancel_last_goal(); */
    // after goal is asynchronously canceled, reset last_goal_handle_
    /* void cancel_goal_callback(rclcpp_action::Client<RobotMove>::CancelResponse::SharedPtr); */
    /* GoalHandleRobotMove::SharedPtr last_goal_handle_; */

    rclcpp::TimerBase::SharedPtr get_task_timer_;
>>>>>>> 0192610ac9... begin writing agent communication
    void get_task();
    rclcpp::TimerBase::SharedPtr get_task_timer_;
    // note that this is our RobotIntent struct (robot_intent.hpp), not a
    // pre-generated ROS msg type
    RobotIntent last_task_;

    // const because should never be changed, but initializer list will allow
    // us to set this once initially
    const int robot_id_;

};  // class AgentActionClient

}  // namespace strategy
