#pragma once

#include <string>

/**
 * \file This file contains all the topic name strings to avoid typos.
 */

namespace config_server::topics {
constexpr auto kGameSettingsPub = "config/game_settings";
constexpr auto kFieldDimensionsPub = "config/field_dimensions";

constexpr auto kGameSettingsSrv = "config/set_game_settings";
constexpr auto kFieldDimensionsSrv = "config/set_field_dimensions";
}  // namespace config_server::topics

namespace sim::topics {

constexpr auto kSimPlacementSrv = "sim/placement";

}  // namespace sim::topics

namespace viz::topics {

constexpr auto kDebugDrawPub = "viz/debug_draw";

}  // namespace viz::topics

namespace referee::topics {
constexpr auto kGameStatePub = "referee/game_state";
constexpr auto kOurInfoPub = "referee/our_info";
constexpr auto kTheirInfoPub = "referee/their_info";
constexpr auto kRefereeRawPub = "referee/raw_protobuf";
constexpr auto kGoaliePub = "referee/our_goalie";
constexpr auto kTeamColorPub = "referee/team_color";

constexpr auto kQuickCommandsSrv = "referee/quick_commands";
constexpr auto kQuickRestartSrv = "referee/quick_restart";
}  // namespace referee::topics

namespace vision_receiver::topics {
constexpr auto kRawProtobufPub = "vision_receiver/raw_protobuf";
constexpr auto kDetectionFramePub = "vision_receiver/detection_frame";
}  // namespace vision_receiver::topics

namespace vision_filter::topics {
constexpr auto kWorldStatePub = "vision_filter/world_state";
}  // namespace vision_filter::topics

namespace gameplay::topics {

static inline std::string robot_intent_pub(int robot_id) {
    return "gameplay/robot_intent/robot_" + std::to_string(robot_id);
}

}  // namespace gameplay::topics

namespace planning::topics {

constexpr auto kGlobalObstaclesPub = "planning/global_obstacles";
constexpr auto kGoalZoneObstacles = "planning/goal_zone_obstacles";

static inline std::string trajectory_pub(int robot_id) {
    return "planning/trajectory/robot_" + std::to_string(robot_id);
}

}  // namespace planning::topics

namespace control {

namespace topics {

static inline std::string manipulator_setpoint_pub(int robot_id) {
    return "control/manipulator_setpoint/robot_" + std::to_string(robot_id);
}

static inline std::string motion_setpoint_pub(int robot_id) {
    return "control/motion_setpoint/robot_" + std::to_string(robot_id);
}

static inline std::string robot_controlled_pub(int robot_id) {
    return "control/robot_controlled/robot_" + std::to_string(robot_id);
}

}  // namespace topics

namespace params {

constexpr auto kMotionControlParamModule = "motion_control";
}  // namespace params

}  // namespace control

namespace radio::topics {

static inline std::string robot_status_pub(int robot_id) {
    return "radio/robot_status/robot_" + std::to_string(robot_id);
}

}  // namespace radio::topics
