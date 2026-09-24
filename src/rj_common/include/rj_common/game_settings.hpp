#pragma once

#include <optional>

#include <rj_common/referee_enums.hpp>
#include <rj_common/time.hpp>
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
    bool request_blue_team = true;
    int request_goalie_id = 0;

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

namespace rclcpp {

template <>
struct TypeAdapter<GameSettings, GameSettings::Msg> {
    using is_specialized = std::true_type;
    using custom_type = GameSettings;
    using ros_message_type = GameSettings::Msg;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        rj_convert::convert_to_ros(source.simulation, &destination.simulation);
        rj_convert::convert_to_ros(source.request_blue_team, &destination.request_blue_team);
        rj_convert::convert_to_ros(source.request_goalie_id, &destination.request_goalie_id);
        rj_convert::convert_to_ros(source.defend_plus_x, &destination.defend_plus_x);
        rj_convert::convert_to_ros(source.use_our_half, &destination.use_our_half);
        rj_convert::convert_to_ros(source.use_their_half, &destination.use_their_half);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rj_convert::convert_from_ros(source.simulation, &destination.simulation);
        rj_convert::convert_from_ros(source.request_blue_team, &destination.request_blue_team);
        rj_convert::convert_from_ros(source.request_goalie_id, &destination.request_goalie_id);
        rj_convert::convert_from_ros(source.defend_plus_x, &destination.defend_plus_x);
        rj_convert::convert_from_ros(source.use_our_half, &destination.use_our_half);
        rj_convert::convert_from_ros(source.use_their_half, &destination.use_their_half);
    }
};


}  // namespace rclcpp