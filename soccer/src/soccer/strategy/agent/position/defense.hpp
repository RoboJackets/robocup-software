#pragma once

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"
#include "waller.hpp"
#include "marker.hpp"

namespace strategy {

/*
 * The Defense position handles general defensive tasks, like intercepting
 * passes, walling in front of our goal, and fighting for possession.
 */
class Defense : public Position {
public:
    Defense(int r_id);
    ~Defense() override = default;

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
    int move_ct_ = 0;

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
        IDLING,        // simply staying in place
        JOINING_WALL,  // send message to find its place in the wall
        WALLING,       // participating in the wall
        SEARCHING,     // moving around on the field to do something
        RECEIVING,     // physically intercepting the ball from a pass
        PASSING,       // physically kicking the ball towards another robot
        FACING,        // turning to face the passing robot
        MARKING,
        ENTERING_MARKING,
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
