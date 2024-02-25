#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "game_state.hpp"
#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "strategy/agent/position/defense.hpp"
#include "strategy/agent/position/goal_kicker.hpp"
#include "strategy/agent/position/goalie.hpp"
#include "strategy/agent/position/offense.hpp"
#include "strategy/agent/position/penalty_player.hpp"
#include "strategy/agent/position/position.hpp"

namespace strategy {

/*
 * A Position that exclusively delegates to another Position.
 * Used to let agents decide which position is best to play based on situation.
 */
class RobotFactoryPosition : public Position {
public:
    RobotFactoryPosition(int r_id);
    ~RobotFactoryPosition() override = default;

    // Copy and move for this class is really annoying because it contains
    // a unique_ptr to an abstract class.
    RobotFactoryPosition(const RobotFactoryPosition& other) = delete;
    RobotFactoryPosition(RobotFactoryPosition&& other) = delete;
    RobotFactoryPosition& operator=(const RobotFactoryPosition& other) = delete;
    RobotFactoryPosition& operator=(RobotFactoryPosition&& other) = delete;

    std::optional<RobotIntent> get_task(WorldState& world_state, FieldDimensions& field_dimensions,
                                        PlayState& play_state) override;

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;

    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    std::deque<communication::PosAgentRequestWrapper> send_communication_request() override;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

    void set_is_done() override;
    void die() override;
    void revive() override;

private:
    std::unique_ptr<Position> current_position_;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    static std::pair<int, double> get_closest_kicker(
        const std::unordered_map<int, double>& kicker_distances);

    void set_default_positions(WorldState& world_state, FieldDimensions& field_dimensions);
};

}  // namespace strategy
