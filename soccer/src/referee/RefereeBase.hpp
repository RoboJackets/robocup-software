#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_info.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/game_state.hpp>
#include <rj_msgs/msg/goalie.hpp>

#include "GameState.hpp"
#include "TeamInfo.hpp"
#include "WorldState.hpp"

namespace referee {

using GameStateMsg = rj_msgs::msg::GameState;
using GoalieMsg = rj_msgs::msg::Goalie;
using TeamColorMsg = rj_msgs::msg::TeamColor;
using TeamInfoMsg = rj_msgs::msg::TeamInfo;
using GameStateMsg = rj_msgs::msg::GameState;

class RefereeBase : public rclcpp::Node {
public:
protected:
    RefereeBase(const std::string& name);

    void play();
    void stop();
    void halt();
    void setup();
    void ready();
    void restart(GameState::Restart type, bool blue_restart);
    void ball_placement(Geometry2d::Point point, bool blue_placement);

    void set_period(GameState::Period period);

    void set_team_name(std::string name);
    void set_team_info(const TeamInfo& blue, const TeamInfo& yellow);
    void set_team_color(bool is_blue);

    void set_goalie(uint8_t goalie_id);

    // TODO(Kyle): Implement kick detector
    void capture_ready_point(const Geometry2d::Point& ball_position) {
        _capture_ready_point = ball_position;
    }
    void spin_kick_detector(const BallState& ball_position) {
        if (!_state.inReadyState()) {
            _capture_ready_point = std::nullopt;
        }

        if (_capture_ready_point.has_value()) {
            constexpr double kMovedRadius = 0.1;
            if (!_capture_ready_point->nearPoint(ball_position.position, kMovedRadius)) {
                play();
            }
        }
    }

    void send();

private:
    std::string _our_name;

    TeamInfo _blue_info;
    TeamInfo _yellow_info;
    bool _team_info_valid = false;

    GameState _state;
    bool _blue_restart = false;
    bool _state_valid = false;

    bool _goalie_valid = false;

    bool _blue_team = false;
    bool _blue_team_valid = false;

    rclcpp::Publisher<TeamColorMsg>::SharedPtr _team_color_pub;
    rclcpp::Publisher<GoalieMsg>::SharedPtr _goalie_id_pub;
    rclcpp::Publisher<TeamInfoMsg>::SharedPtr _our_team_info_pub;
    rclcpp::Publisher<TeamInfoMsg>::SharedPtr _their_team_info_pub;
    rclcpp::Publisher<GameStateMsg>::SharedPtr _game_state_pub;

    void update_team_color_from_names();

    // Kick detector information
    std::optional<Geometry2d::Point> _capture_ready_point;
};

} // namespace referee