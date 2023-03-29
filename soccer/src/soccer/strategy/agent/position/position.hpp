#pragma once

#include <cstdlib>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/coach_state.hpp>
#include <rj_msgs/msg/global_override.hpp>
#include <rj_common/field_dimensions.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"
#include "robot_intent.hpp"
#include "world_state.hpp"

// Communication
#include "../communication/communication.hpp"

// Requests
#include <rj_msgs/msg/pass_request.hpp>
#include <rj_msgs/msg/position_request.hpp>
#include <rj_msgs/msg/test_request.hpp>

// Responses
#include <rj_msgs/msg/acknowledge.hpp>
#include <rj_msgs/msg/pass_response.hpp>
#include <rj_msgs/msg/position_request.hpp>
#include <rj_msgs/msg/test_response.hpp>

// tell compiler this class exists, but no need to import the whole header
class AgentActionClient;

namespace strategy {
/*
 * Position is an abstract superclass. Its subclasses handle strategy logic.
 * The goal is to isolate the strategy logic from the ROS interfacing. This
 * allows us to swap Position classes at runtime. (Google "Strategy Design
 * Pattern" for more info.)
 *
 * A good analogy is how the Planner Node uses the various Planner objects
 * (PathTargetPlanner, etc.). The Planner objects take in a plan request and
 * output a Trajectory. Planner objects don't know anything about ROS; that is
 * all handled by the Planner Node.
 */
class Position {
public:
    Position(int r_id);
    virtual ~Position() = default;

    /**
     * @brief return a RobotIntent to be sent to PlannerNode by AC; nullopt
     * means no new task requested.
     *
     * Creates a RobotIntent with the right robot ID, then returns EmptyMotionCommand
     * if world_state is invalid, then delegates to derived classes.
     *
     * Uses the Template Method + non-virtual interface:
     * https://www.sandordargo.com/blog/2022/08/24/tmp-and-nvi
     */
    std::optional<RobotIntent> get_task();

    // communication with AC
    void update_world_state(WorldState world_state);
    void update_coach_state(rj_msgs::msg::CoachState coach_state);
    void update_field_dimensions(FieldDimensions field_dimensions);
    const std::string get_name();

    /**
     * @brief setter for time_left_
     */
    void set_time_left(double time_left);

    /**
     * @brief setter for is_done_
     *
     * Outside classes can only set to true, Position/derived classes can clear
     * with check_is_done().
     */
    void set_is_done();

    /**
     * @brief setter for goal_canceled_
     *
     * Outside classes can only set to true, Position/derived classes can clear
     * with check_is_done().
     */
    void set_goal_canceled();

    // Agent-to-Agent communication
    /**
     * @brief Send the intended communication request through the agent action client.
     *
     * @return communication::PosAgentRequestWrapper the request to be sent
     */
    communication::PosAgentRequestWrapper send_communication_request();

    /**
     * @brief Receive the response from a sent request.
     *
     * @param response the response received from the previously sent communication
     */
    virtual void receive_communication_response(communication::AgentPosResponseWrapper response);

    /**
     * @brief Receives a communication request from another robot before handling the request
     * and sending a response in return.
     *
     * @param request the request received from the other robot
     * @return communication::PosAgentResponseWrapper this robot's response to the other robot.
     */
    virtual communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request);
    // END Agent-to-Agent Communication

protected:
    // should be overriden in subclass constructors
    std::string position_name_{"Position"};

    // field for tell_time_left() above
    double time_left_{};
    bool is_done_{};
    bool goal_canceled_{};

    // fields for coach_state
    // TODO: this is not thread-safe, does it need to be?
    // (if so match world_state below)
    int match_situation_{};  // TODO: this is an enum, get from coach_node
    bool our_possession_{};
    rj_msgs::msg::GlobalOverride global_override_{};

    FieldDimensions field_dimensions_ = FieldDimensions::kDefaultDimensions;

    /*
     * @return thread-safe ptr to most recent world_state
     */
    [[nodiscard]] WorldState* world_state();

    /*
     * @brief assert world_state is valid before using it in get_task().
     *
     * Since ROS nodes launch in a random order, often the agents will launch
     * before they have any world_state info to act on. Thus, we must return
     * NOPs to the robots until vision_filter node starts up.
     *
     * @return false if world_state is invalid (nullptr), true otherwise
     */
    bool assert_world_state_valid();

    /*
     * @brief getter for is_done that clears the flag before returning
     * @return value of is_done before being cleared
     */
    bool check_is_done();

    /*
     * @brief getter for goal_canceled that clears the flag before returning
     * @return value of goal_canceled before being cleared
     */
    bool check_goal_canceled();

    // const because should never be changed, but initializer list will allow
    // us to set this once initially
    const int robot_id_;

    // Request
    communication::PosAgentRequestWrapper communication_request_;

private:
    // private to avoid allowing WorldState to be accessed directly by derived
    // classes (must use thread-safe getter)
    WorldState last_world_state_;
    mutable std::mutex world_state_mutex_;

    /**
     * @brief allow derived classes to change behavior of get_task(). See
     * get_task() above.
     * @param intent a blank RobotIntent with this robot's ID filled in already
     */
    virtual std::optional<RobotIntent> derived_get_task(RobotIntent intent) = 0;
};

}  // namespace strategy
