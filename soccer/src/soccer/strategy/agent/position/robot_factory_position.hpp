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

    std::optional<RobotIntent> get_task(WorldState& world_state,
                                        FieldDimensions& field_dimensions) override;

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;

    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    std::optional<communication::PosAgentRequestWrapper> send_communication_request() override;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

    void set_is_done() override;
    void die() override;
    void revive() override;

    inline void update_play_state(const PlayState& play_state) override {
        current_position_->update_play_state(play_state);
    }

    inline void update_field_dimensions(const FieldDimensions& field_dimensions) override {
        current_position_->update_field_dimensions(field_dimensions);
    }

    inline void update_alive_robots(std::vector<u_int8_t> alive_robots) override {
        current_position_->update_alive_robots(alive_robots);
    }

    inline const std::string get_name() override { current_position_->get_name(); }

    inline void set_time_left(double time_left) override {
        current_position_->set_time_left(time_left);
    }

    inline void set_goal_canceled() override { current_position_->set_goal_canceled(); }

    inline void send_direct_pass_request(std::vector<u_int8_t> target_robots) override {
        current_position_->send_direct_pass_request(target_robots);
    }

    inline void broadcast_direct_pass_request() override {
        current_position_->broadcast_direct_pass_request();
    }

    inline communication::PassResponse receive_pass_request(
        communication::PassRequest pass_request) override {
        current_position_->receive_pass_request(pass_request);
    }

    inline void send_pass_confirmation(u_int8_t target_robot) override {
        current_position_->send_pass_confirmation(target_robot);
    }

    inline void set_goalie_id(int goalie_id) override {
        current_position_->set_goalie_id(goalie_id);
    }

private:
    std::unique_ptr<Position> current_position_;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};

}  // namespace strategy
