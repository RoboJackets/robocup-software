#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/game_state.hpp>
#include <rj_msgs/msg/goalie.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/team_info.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "game_state.hpp"
#include "team_info.hpp"
#include "world_state.hpp"
#include "config_client/config_client.hpp"

namespace referee {

using GameStateMsg = rj_msgs::msg::GameState;
using GoalieMsg = rj_msgs::msg::Goalie;
using TeamColorMsg = rj_msgs::msg::TeamColor;
using TeamInfoMsg = rj_msgs::msg::TeamInfo;

constexpr auto kRefereeParamModule = "referee";

/**
 * @brief Base class for both types of referee. Handles sending ROS messages to
 * the relevant channels (when necessary).
 *
 * There are four main topic types published by this module:
 *  - GameState: contains information relevant to how the game should
 * _currently_ be played (playing/halt/ready/stop, restart, etc).
 *  - TeamInfo (ours and theirs): team names, scores, and card information.
 *  - TeamColor: our team color.
 *  - Goalie: which robot is currently used as our goalie.
 *
 * @details All topics are latched using transient_local() with depth 1.
 */
class RefereeBase : public rclcpp::Node {
public:
protected:
    /**
     * @brief Constructor for RefereeBase
     * @param name the name of the node.
     */
    RefereeBase(const std::string& name);

    /**
     * @brief Change to the "playing" state, representing normal gameplay.
     */
    void play();

    /**
     * @brief Move to the "stop" state.
     */
    void stop();

    /**
     * @brief Move to the "halt" state, where all robots are forced to stop.
     */
    void halt();

    /**
     * @brief Move to the "setup" state to prepare for a restart.
     */
    void setup();

    /**
     * @brief Move to the "ready" state, for while a restart is occuring.
     */
    void ready();

    /**
     * @brief Do a restart. This does not change the "state", only the current
     * restart.
     */
    void restart(GameState::Restart type, bool blue_restart);

    /**
     * @brief Do ball placement. This is currently unimplemented.
     * @param point The goal point for placement.
     * @param blue_placement Who is placing the ball.
     */
    void ball_placement(rj_geometry::Point point, bool blue_placement);

    void set_period(GameState::Period period);
    void set_stage_time_left(RJ::Seconds stage_time_left);
    void set_team_name(const std::string& name);

    /**
     * @brief Set the information for blue/yellow teams, and set our color
     * accordingly if either color matches our preferred name.
     *
     * @param blue
     * @param yellow
     */
    void set_team_info(const TeamInfo& blue, const TeamInfo& yellow);
    void set_team_color(bool is_blue);

    void set_goalie(uint8_t goalie_id);

    bool should_capture() const { return (state_.in_ready_state() || state_.in_setup_state()) && !state_.placement(); }

    void capture_ready_point(const rj_geometry::Point& ball_position) {
        std::cout << "State: " << state_.state << std::endl;
        if (should_capture()) {
            std::cout << "Capturing ball position" << std::endl;
            capture_ready_point_ = ball_position;
        }
    }

    void spin_kick_detector(const BallState& ball_position) {
        if (!should_capture() && capture_ready_point_.has_value()) {
            capture_ready_point_ = std::nullopt;
            std::cout << "Clearing ball position" << std::endl;
        }

        if (should_capture() && capture_ready_point_.has_value()) {
            std::cout << "Comparing ball position" << std::endl;
            constexpr double kMovedRadius = 0.1;
            if (!capture_ready_point_->near_point(ball_position.position, kMovedRadius)) {
                play();
            }
        }
    }

    [[nodiscard]] bool our_restart() const { return state_.our_restart; }

    /**
     * @brief Send any updated messages.
     */
    void send();

private:
    std::string our_name_;

    /**
     * @brief Current team info for both teams.
     *
     * @details This is indexed by blue/yellow rather than ours/theirs to avoid
     * transient issues when our team color switches.
     */
    TeamInfo blue_info_;
    TeamInfo yellow_info_;
    bool team_info_valid_ = false;

    /**
     * @brief Current game state.
     */
    GameState state_;
    bool blue_restart_ = false;
    bool state_valid_ = false;

    bool goalie_valid_ = false;

    /**
     * @brief Our current team color.
     */
    bool blue_team_ = false;
    bool blue_team_valid_ = false;

    /**
     * @brief Whether we at least have an accurate blue_team value.
     *
     * @details If we don't we shouldn't send out anything at all.
     */
    bool has_any_info_ = false;

    rclcpp::Publisher<TeamColorMsg>::SharedPtr team_color_pub_;
    rclcpp::Publisher<GoalieMsg>::SharedPtr goalie_id_pub_;
    rclcpp::Publisher<TeamInfoMsg>::SharedPtr our_team_info_pub_;
    rclcpp::Publisher<TeamInfoMsg>::SharedPtr their_team_info_pub_;
    rclcpp::Publisher<GameStateMsg>::SharedPtr game_state_pub_;
    rclcpp::Subscription<WorldState::Msg>::SharedPtr world_state_sub_;

    bool did_setup_end_ = false;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    GameStateMsg gamestate_msg;
    rclcpp::TimerBase::SharedPtr gamestate_pub_timer_;

    /**
     * @brief Update the team color from the names currently available in the
     * blue and yellow team information.
     */
    void update_team_color_from_names();

    // Kick detector information
    std::optional<rj_geometry::Point> capture_ready_point_;

    params::LocalROS2ParamProvider param_provider_;

    bool saved_restart_valid_ = false;
    bool saved_restart_team_blue_ = false;
    GameState::Restart saved_restart_type_ = GameState::Restart::None;
    rj_geometry::Point last_ball_position_;

    config_client::ConfigClient config_client_;

    /**
     * @brief Returns the team angle
     * @return The team angle.
     */
    [[nodiscard]] double team_angle() const {
        const bool defend_plus_x = config_client_.game_settings().defend_plus_x;
        return defend_plus_x ? -M_PI_2 : M_PI_2;
    }

    /**
     * @brief Returns the transform from the world to the team.
     * @return The transform from the world to the team frame.
     */
    [[nodiscard]] rj_geometry::TransformMatrix world_to_team() const {
        rj_geometry::TransformMatrix world_to_team = rj_geometry::TransformMatrix::translate(
            0, config_client_.field_dimensions().length / 2.0f);
        world_to_team *= rj_geometry::TransformMatrix::rotate(static_cast<float>(team_angle()));
        return world_to_team;
    }
};

}  // namespace referee
