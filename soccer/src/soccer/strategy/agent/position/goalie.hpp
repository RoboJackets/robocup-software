#pragma once

#include <cmath>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>
#include <rj_msgs/msg/goalie_idle_motion_command.hpp>
#include <rj_msgs/msg/intercept_motion_command.hpp>
#include <rj_msgs/msg/path_target_motion_command.hpp>

#include "planning/planner/intercept_planner.hpp"
#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

namespace strategy {

/*
 * The Goalie position handles goalie behavior: blocking shots, passing to teammates, and clearing
 * the ball.
 */
class Goalie : public Position {
public:
    Goalie(int r_id);
    ~Goalie() override = default;

    std::optional<rj_msgs::msg::RobotIntent> get_task() override;

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

private:
    // temp
    int send_idle_ct_ = 0;

    // see Position superclass
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // possible states of the Goalie
    enum State { BLOCKING, CLEARING, IDLING, BALL_NOT_FOUND };

    /*
     * @return true if ball is heading towards goal at some minimum speed threshold
     */
    bool shot_on_goal_detected(WorldState* world_state);

    /*
     * Provide an updated state for the Goalie state machine based
     * on the current world state.
     *
     * e.g. when shot on goal detected, Goalie should intercept the
     * ball
     *
     * @return current State to set Goalie's state machine to
     */
    State update_state();

    /*
     * Based on the Goalie's current state, send a motion_command
     * to the planner node.
     *
     * @param intent RobotIntent to add the desired motion_command
     * to
     *
     * @return matching return type of derived_get_task() (see
     * above)
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of Goalie (state machine)
    State latest_state_ = IDLING;

    // TEST CODE //
    void test_unicast_request();
    communication::TestRequest unicast_test_request_{100};

    void test_multicast_request();
    communication::TestRequest multicast_test_request_{100};

    void test_broadcast_request();
    communication::TestRequest broadcast_test_request_{100};

    void test_anycast_request();
    communication::TestRequest anycast_test_request_{100};

    void test_timeout_request();
    communication::PassRequest timeout_test_request_{100};
    // END TEST CODE //
};

}  // namespace strategy
