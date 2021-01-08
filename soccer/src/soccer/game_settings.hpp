#pragma once

#include <optional>

#include <rj_common/referee_enums.hpp>
#include <rj_common/team_color.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/game_settings.hpp>

/**
 * Settings for the game, set by MainWindow to be consumed by the rest of the
 * soccer program. This includes playbooks and general settings.
 */
struct GameSettings {
    using Msg = rj_msgs::msg::GameSettings;

    GameSettings() = default;

    // Whether or not we're in simulation.
    bool simulation = true;

    // Requests. These can be overridden by the referee if it's enabled
    TeamColor requested_team_color = TeamColor::kBlue;
    RobotId request_goalie_id = 0;

    // Defend the plus-x direction in vision
    bool defend_plus_x = true;

    bool use_our_half = true;
    bool use_their_half = true;

    bool paused = false;

    struct JoystickConfig {
        int manual_id = -1;
        bool damped_translation = true;
        bool damped_rotation = true;
        bool use_kick_on_break_beam = false;
        bool use_field_oriented_drive = false;
    };

    JoystickConfig joystick_config;
};

namespace rj_convert {

template <>
struct RosConverter<GameSettings, GameSettings::Msg> {
    static GameSettings::Msg to_ros(const GameSettings& from) {
        GameSettings::Msg to;
        convert_to_ros(from.simulation, &to.simulation);
        convert_to_ros(from.requested_team_color, &to.requested_team_color);
        convert_to_ros(static_cast<int>(from.request_goalie_id), &to.request_goalie_id);
        convert_to_ros(from.defend_plus_x, &to.defend_plus_x);
        convert_to_ros(from.use_our_half, &to.use_our_half);
        convert_to_ros(from.use_their_half, &to.use_their_half);
        return to;
    }

    static GameSettings from_ros(const GameSettings::Msg& from) {
        GameSettings to;
        convert_from_ros(from.simulation, &to.simulation);
        convert_from_ros(from.requested_team_color, &to.requested_team_color);
        convert_from_ros(static_cast<RobotId>(from.request_goalie_id), &to.request_goalie_id);
        convert_from_ros(from.defend_plus_x, &to.defend_plus_x);
        convert_from_ros(from.use_our_half, &to.use_our_half);
        convert_from_ros(from.use_their_half, &to.use_their_half);
        return to;
    }
};

ASSOCIATE_CPP_ROS(GameSettings, GameSettings::Msg);

}  // namespace rj_convert