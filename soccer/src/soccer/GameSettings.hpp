#pragma once

#include <optional>
#include <rj_common/RefereeEnums.hpp>
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
    bool requestBlueTeam = true;
    int requestGoalieID = 0;

    // Defend the plus-x direction in vision
    bool defendPlusX = true;

    bool use_our_half = true;
    bool use_their_half = true;

    bool paused = false;

    struct JoystickConfig {
        int manualID = -1;
        bool dampedTranslation = true;
        bool dampedRotation = true;
        bool useKickOnBreakBeam = false;
        bool useFieldOrientedDrive = false;
    };

    JoystickConfig joystick_config;
};

namespace rj_convert {

template <>
struct RosConverter<GameSettings, GameSettings::Msg> {
    static GameSettings::Msg to_ros(const GameSettings& from) {
        GameSettings::Msg to;
        convert_to_ros(from.simulation, &to.simulation);
        convert_to_ros(from.requestBlueTeam, &to.request_blue_team);
        convert_to_ros(from.requestGoalieID, &to.request_goalie_id);
        convert_to_ros(from.defendPlusX, &to.defend_plus_x);
        convert_to_ros(from.use_our_half, &to.use_our_half);
        convert_to_ros(from.use_their_half, &to.use_their_half);
        return to;
    }

    static GameSettings from_ros(const GameSettings::Msg& from) {
        GameSettings to;
        convert_to_ros(from.simulation, &to.simulation);
        convert_to_ros(from.request_blue_team, &to.requestBlueTeam);
        convert_to_ros(from.request_goalie_id, &to.requestGoalieID);
        convert_to_ros(from.defend_plus_x, &to.defendPlusX);
        convert_to_ros(from.use_our_half, &to.use_our_half);
        convert_to_ros(from.use_their_half, &to.use_their_half);
        return to;
    }
};

ASSOCIATE_CPP_ROS(GameSettings, GameSettings::Msg);

}  // namespace rj_convert