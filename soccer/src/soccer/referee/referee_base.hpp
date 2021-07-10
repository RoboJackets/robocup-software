#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/match_state.hpp>
#include <rj_msgs/msg/play_state.hpp>
#include <rj_msgs/msg/goalie.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/team_info.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "config_client/config_client.hpp"
#include "game_state.hpp"
#include "team_info.hpp"
#include "world_state.hpp"

namespace referee {

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
     * Handle a state transition. This should be called whenever we get a new command including
     * state, restart, or ball placement.
     *
     * !! This should _not_ be called repeatedly for the same command !!
     *
     * This is always the play state for the YELLOW team (i.e. yellow restarts correspond to OURS)
     */
    void set_play_state(const PlayState& state) {
        // Some commands (basically anything that puts us into Setup, like direct/indirect kicks and
        // normal starts, need us to know where the ball started when the state was entered.
        if (state.wait_for_kick()) {
            // We only change the placement point if the command has changed.
            // Edge case: change the placement point if we didn't already have one (like it
            // disappeared from vision when we first got the command).
            if (yellow_play_state_ != state || !kick_watcher_capture_position_.has_value()) {
                kick_watcher_capture_position_ = last_ball_position_;
            }
        } else {
            kick_watcher_capture_position_ = std::nullopt;
        }

        yellow_play_state_ = state;
    }

    void set_period(MatchState::Period period);
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

    /**
     * @brief Set the goalie ID for our team. We will always defer to the referee if we have
     * information from ref packets.
     */
    void override_goalie(uint8_t goalie_id);

    /**
     * @brief Spin the kick detector and update the ball position.
     * @details Triggered every vision frame.
     * @returns True if there was a state transition
     */
    bool spin_kick_detector(const std::optional<rj_geometry::Point>& ball_position) {
        // We need this for capturing points.
        last_ball_position_ = ball_position;

        // Run the kick watcher if we need to (only when we're waiting for a kick to proceed).
        if (yellow_play_state_.wait_for_kick() && kick_watcher_capture_position_ && ball_position) {
            constexpr double kMovedRadius = 0.1;
            if (!kick_watcher_capture_position_->near_point(*ball_position, kMovedRadius)) {
                set_play_state(yellow_play_state_.advanced_from_kick());
                return true;
            }
        }

        return false;
    }

    /**
     * @brief Send out all valid messages.
     */
    void send();

    PlayState yellow_play_state() const { return yellow_play_state_; }

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

    /// @brief Current state of the match. Includes period and time remaining.
    MatchState match_state_{};

    /// @brief Current state of play. Includes information about whether we should be actively
    /// playing as well as restart/placement information from the yellow team's perspective.
    /// @details This is always for the yellow team to prevent confusion when changing team colors.
    /// It is resolved to the correct team when it is published.
    PlayState yellow_play_state_ = PlayState::halt();

    /// @brief Point captured when entering Setup by the kick detector.
    std::optional<rj_geometry::Point> kick_watcher_capture_position_ = std::nullopt;

    /**
     * @brief Our current team color.
     */
    bool blue_team_ = false;

    /**
     * @brief Whether we at least have an accurate blue_team value.
     *
     * @details If we don't we shouldn't send out anything at all.
     */
    bool has_any_info_ = false;

    /**
     * @brief Update the team color from the names currently available in the
     * blue and yellow team information.
     */
    void update_team_color_from_names();

    /**
     * @brief The latest ball position from vision, updated continuously.
     * @details Should be nullopt whenever we lose the ball on vision.
     */
    std::optional<rj_geometry::Point> last_ball_position_;

    params::LocalROS2ParamProvider param_provider_;

    config_client::ConfigClient config_client_;

    rclcpp::Publisher<TeamColorMsg>::SharedPtr team_color_pub_;
    rclcpp::Publisher<GoalieMsg>::SharedPtr goalie_id_pub_;
    rclcpp::Publisher<TeamInfoMsg>::SharedPtr our_team_info_pub_;
    rclcpp::Publisher<TeamInfoMsg>::SharedPtr their_team_info_pub_;
    rclcpp::Publisher<PlayState::Msg>::SharedPtr play_state_pub_;
    rclcpp::Publisher<MatchState::Msg>::SharedPtr match_state_pub_;
    rclcpp::Subscription<WorldState::Msg>::SharedPtr world_state_sub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};

}  // namespace referee
