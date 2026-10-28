#pragma once

#include <memory>
#include <optional>

#include <rj_common/referee_enums.hpp>
#include <rj_common/time.hpp>

#include <rj_msgs/msg/game_settings.hpp>

/**
 * Settings for the game, set by MainWindow to be consumed by the rest of the
 * soccer program. This includes playbooks and general settings.
 */
struct GameSettings {
    using Msg = rj_msgs::msg::GameSettings;
    using SharedPtr = std::shared_ptr<GameSettings>;
    using ConstSharedPtr = std::shared_ptr<const GameSettings>;

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

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination.simulation = source.simulation;
        destination.request_blue_team = source.request_blue_team;
        destination.request_goalie_id = source.request_goalie_id;
        destination.defend_plus_x = source.defend_plus_x;
        destination.use_our_half = source.use_our_half;
        destination.use_their_half = source.use_their_half;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination.simulation = source.simulation;
        destination.request_blue_team = source.request_blue_team;
        destination.request_goalie_id = source.request_goalie_id;
        destination.defend_plus_x = source.defend_plus_x;
        destination.use_our_half = source.use_our_half;
        destination.use_their_half = source.use_their_half;
    }
};

}  // namespace rclcpp
