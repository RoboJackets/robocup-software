#pragma once

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "marker.hpp"
#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"
#include "waller.hpp"

namespace strategy {

/*
 * The Defense position handles general defensive tasks, like intercepting
 * passes, walling in front of our goal, and fighting for possession.
 */
class Defense : public Position {
public:
    Defense(int r_id);
    ~Defense() override = default;
    Defense(const Position& other);

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;
    std::string get_current_state() override;

    void die() override;
    void revive() override;

private:
    const rj_geometry::Point clear_point_{0.0, 4.5};

    // static constexpr int kMaxWallers{6};
    static constexpr int kMaxWallers{
        static_cast<int>(kNumShells)};  // This effectively turns off marking

    /**
     * @brief The derived_get_task method returns the task for the defensive robot
     *  to do based on the game situation. The method will continuously look to assign
     *  the robot to a defensive role, allowing us to quickly swap roles through
     *  gametime. Roles aim to include things such as waller, blocker, etc.
     *
     * @param [RobotIntent intent] [RobotIntent of the Defensive Robot]
     * @return [RobotIntent with next target point for the robot]
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        IDLING,            // simply staying in place
        JOINING_WALL,      // send message to find its place in the wall
        WALLING,           // participating in the wall
        SEARCHING,         // moving around on the field to do something
        RECEIVING,         // physically intercepting the ball from a pass
        PASSING,           // physically kicking the ball towards another robot
        FACING,            // turning to face the passing robot
        MARKING,           // Following closely to an offense robot
        ENTERING_MARKING,  // Choosing/waiting for a robot to mark
        STEALING,          // wall stealing
        CLEARING,
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    /**
     * @brief Sends a JoinWallRequest in broadcast to the other robots
     */
    void send_join_wall_request();

    /**
     * @brief Sends a LeaveWallRequest to each of the robots in walling_robots_.
     */
    void send_leave_wall_request();

    /**
     * @return if the current state has timed out
     * wall steal
     */
    /*bool timed_out() const {
        // Defined here so it can be inlined
        using namespace std::chrono_literals;


        return (max_time > 0s) && (last_time_ + max_time < RJ::now());
    };*/

    /**
     * @return distance from this agent to ball
     * wall steal
     */
    double distance_to_ball() const {
        return last_world_state_->ball.position.dist_to(
            last_world_state_->get_robot(true, robot_id_).pose.position());
    };

    /**
     * @brief This FSM has timeouts for certain states.
     * Ideally, these would not be necessary; as planners get more sophisticated
     * they should not get "stuck". However, empirically, the offense FSM in particular
     * has been observed to deadlock often.
     *
     * One common case is when waiting for a receiver to accept a pass; if no receiver responds,
     * the timeout is necessary. In the future receivers may be able to respond in the negative
     * instead of ignoring the request.
     *
     * The timeouts are a safety mechanism, and should not be the primary reason for a
     * state transition. They are set relatively high for this reason.
     *
     * @return the maximum duration to stay in a given state, or -1 if there is no maximum.
     *
     */
    static constexpr RJ::Seconds timeout(State s) {
        switch (s) {
            case PASSING:
                return RJ::Seconds{5};
            case STEALING:
                return RJ::Seconds{10};
        }
    }


    // The time at which the last state started.
    RJ::Time last_time_;

    /**
     * @brief Adds the new waller to this robot's list of wallers and updates this robot's position
     * in the wall.
     *
     * @param join_request the request received from another robot about joining the wall
     * @return communication::JoinWallResponse A confirmation for the other robot to join the wall
     * with this robot's ID
     */
    communication::JoinWallResponse handle_join_wall_request(
        communication::JoinWallRequest join_request);

    /**
     * @brief Removes a given robot from this robot's list of wallers.
     *
     * @param leave_request the request from the robot who is leaving the wall
     * @return communication::Acknowledge acknowledgement of the other robot's communication
     */
    communication::Acknowledge handle_leave_wall_request(
        communication::LeaveWallRequest leave_request);

    /**
     * @brief Handles the response from the currently walling robots to find this robot's place in
     * the wall.
     *
     * @param join_response the response from another robot that this robot can join the wall
     */
    void handle_join_wall_response(communication::JoinWallResponse join_response);

    std::vector<u_int8_t> walling_robots_ = {};
    int waller_id_ = -1;

    // current state of the defense agent (state machine)
    int get_waller_id();
    State current_state_ = JOINING_WALL;

    int get_marker_target_id();
    Marker marker_;
};

}  // namespace strategy
