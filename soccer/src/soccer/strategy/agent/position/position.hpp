#pragma once

#include <cstdlib>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/alive_robots.hpp>

#include "game_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"
#include "robot_intent.hpp"
#include "world_state.hpp"

// Communication
#include "../communication/communication.hpp"

// Requests
#include <rj_msgs/msg/ball_in_transit_request.hpp>
#include <rj_msgs/msg/incoming_ball_request.hpp>
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

    virtual std::optional<RobotIntent> get_task(WorldState& world_state,
                                                FieldDimensions& field_dimensions);

    // communication with AC
    void update_play_state(const PlayState& play_state);
    void update_field_dimensions(const FieldDimensions& field_dimensions);
    void update_alive_robots(std::vector<u_int8_t> alive_robots);
    const std::string get_name();

    // returns the current state of the robot
    virtual std::string get_current_state() = 0;

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
    virtual void set_is_done();

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
    virtual std::optional<communication::PosAgentRequestWrapper> send_communication_request();

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

    /**
     * @brief sends a direct pass request to the target robots
     *
     * @param target_robots a vector of robots to send a request to
     */
    void send_direct_pass_request(std::vector<u_int8_t> target_robots);

    void broadcast_direct_pass_request();

    /**
     * @brief receives and handles a pass_request
     *
     * @param pass_request the pass request to handle
     * @return communication::PassResponse a response to the robot as to whether or not it
     * is open for a pass
     */
    communication::PassResponse receive_pass_request(communication::PassRequest pass_request);

    /**
     * @brief tell another robot that this robot will pass to it
     *
     * @param target_robot the robot that will be passed to
     */
    void send_pass_confirmation(u_int8_t target_robot);

    /**
     * @brief acknowledges the pass confirmation from another robot
     *
     * @param incoming_ball_request the request that a ball will be coming to this robot
     * @return communication::Acknowledge acknowledgement that the other robot may pass to this
     * robot
     */
    virtual communication::Acknowledge acknowledge_pass(
        communication::IncomingBallRequest incoming_ball_request);

    /**
     * @brief method called in acknowledge_pass that updates the position to its next state
     *
     */
    virtual void derived_acknowledge_pass(){};

    /**
     * @brief update the robot state to be passing the ball
     *
     * @param robot_id the robot id of the robot to pass the ball to
     */
    virtual void pass_ball(int robot_id);

    /**
     * @brief method called in pass ball that updates the position to its corresponding passing
     * state.
     *
     */
    virtual void derived_pass_ball(){};

    /**
     * @brief the ball is on the way, so the robot should change its state accordingly
     *
     * @param ball_in_transit_request request from the passing robot that they have passed the ball
     * @return communication::Acknowledge response to the passing robot that this robot will be
     * receiving
     */
    virtual communication::Acknowledge acknowledge_ball_in_transit(
        communication::BallInTransitRequest ball_in_transit_request);

    /**
     * @brief method called in acknowledge_ball_in_transit to update the position to its
     * corresponding next state
     *
     */
    virtual void derived_acknowledge_ball_in_transit(){};

    /**
     * @brief When a robot disconnects on field they should call their
     * implementation of die to inform necessary robots that they died.
     *
     */
    virtual void die(){};

    /**
     * @brief When a robot disconnects and comes back to life revive will take care
     * of bringing them back into the correct state.
     *
     */
    virtual void revive(){};

protected:
    // should be overriden in subclass constructors
    std::string position_name_{"Position"};

    // field for tell_time_left() above
    double time_left_{};
    bool is_done_{};
    bool goal_canceled_{};

    // TODO: this is not thread-safe, does it need to be?
    // (if so match world_state below)
    bool our_possession_{};
    FieldDimensions field_dimensions_ = FieldDimensions::kDefaultDimensions;
    PlayState current_play_state_ = PlayState::halt();

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

    // the robot our robot is going to be passing to
    int target_robot_id;

    // the robot our robot will be facing (useful for passing)
    int face_robot_id;

    // the maximum distance from the robot to the ball for the robot to begin
    // chasing the ball
    const double max_receive_distance = 1.0;  // m

    // Whether or not this robot should be chasing the ball on receive
    // set to true when the ball gets close to this robot
    bool chasing_ball = false;

    // Request
    std::optional<communication::PosAgentRequestWrapper> communication_request_;

    // farthest distance the robot is willing to go to receive a pass
    static constexpr double ball_receive_distance_ = 0.1;

    // farthest distance the robot is willing to go before it declares it has lost the ball
    static constexpr double ball_lost_distance_ = 0.5;

    // vector of alive robots from the agent action client
    std::vector<u_int8_t> alive_robots_ = {};

    // true if this robot is alive
    bool alive = false;

    // protected to allow WorldState to be accessed directly by derived
    WorldState* last_world_state_;

private:
    /**
     * @brief allow derived classes to change behavior of get_task(). See
     * get_task() above.
     * @param intent a blank RobotIntent with this robot's ID filled in already
     */
    virtual std::optional<RobotIntent> derived_get_task(RobotIntent intent) = 0;
};

}  // namespace strategy
