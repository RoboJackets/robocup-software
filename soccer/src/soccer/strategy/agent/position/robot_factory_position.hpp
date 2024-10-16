#pragma once

#include <cmath>
#include <string>

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
#include "strategy/agent/position/free_kicker.hpp"
#include "strategy/agent/position/goal_kicker.hpp"
#include "strategy/agent/position/goalie.hpp"
#include "strategy/agent/position/line.hpp"
#include "strategy/agent/position/offense.hpp"
#include "strategy/agent/position/penalty_non_kicker.hpp"
#include "strategy/agent/position/penalty_player.hpp"
#include "strategy/agent/position/pivot_test.hpp"
#include "strategy/agent/position/position.hpp"
#include "strategy/agent/position/smartidling.hpp"
#include "strategy/agent/position/solo_offense.hpp"
#include "strategy/agent/position/zoner.hpp"

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

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;

    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    std::deque<communication::PosAgentRequestWrapper> send_communication_request() override;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

    std::string get_current_state() override;

    void set_is_done() override;
    void die() override;
    void revive() override;

    void update_play_state(const PlayState& play_state) override {
        Position::update_play_state(play_state);
        current_position_->update_play_state(play_state);
    }

    void update_field_dimensions(const FieldDimensions& field_dimensions) override {
        current_position_->update_field_dimensions(field_dimensions);
    }

    void update_alive_robots(std::array<bool, kNumShells> alive_robots) override {
        Position::update_alive_robots(alive_robots);
        current_position_->update_alive_robots(alive_robots);
    }

    const std::string get_name() override { return current_position_->get_name(); }

    void set_time_left(double time_left) override { current_position_->set_time_left(time_left); }

    void set_goal_canceled() override { current_position_->set_goal_canceled(); }

    void set_goalie_id(int goalie_id) override {
        Position::set_goalie_id(goalie_id);
        current_position_->set_goalie_id(goalie_id);
    }

    void send_direct_pass_request(std::vector<u_int8_t> target_robots) override {
        current_position_->send_direct_pass_request(target_robots);
    }

    void broadcast_direct_pass_request() override {
        current_position_->broadcast_direct_pass_request();
    }

    communication::PassResponse receive_pass_request(
        communication::PassRequest pass_request) override {
        return current_position_->receive_pass_request(pass_request);
    }

    void send_pass_confirmation(u_int8_t target_robot) override {
        current_position_->send_pass_confirmation(target_robot);
    }

private:
    std::unique_ptr<Position> current_position_;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    bool am_closest_kicker();

    void set_default_position();

    PlayState last_play_state_{PlayState::halt()};

    void process_play_state();

    void update_position();

    void start_kicker_picker();

    bool have_all_kicker_responses();

    void handle_stop();

    void handle_ready();

    void handle_setup();

    void handle_penalty_playing();

    /**
     * @brief Sets the current position to the parameterized type.
     * Requires the type to have a constructor that takes a reference to the old Position
     * (and to be a subclass of Position)
     */
    template <class Pos>
    void set_current_position() {
        // If we are not currently playing Pos
        // if (current_position_->get_name() == "Defense") {
        //     SPDLOG_INFO("we are never leaving defense :)");
        //     return;
        // }
        if (dynamic_cast<Pos*>(current_position_.get()) == nullptr) {
            // This line requires Pos to implement the constructor Pos(const
            // Position&)
            SPDLOG_INFO("Robot {}: change {}", robot_id_, current_position_->get_name());
            current_position_->die();
            current_position_ = std::make_unique<Pos>(*current_position_);
        }
    }
};

}  // namespace strategy
