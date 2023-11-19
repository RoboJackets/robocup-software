#pragma once

#include <string>

/**
 * \file This file contains all the topic name strings to avoid typos.
 */

namespace config_server::topics {
constexpr auto kGameSettingsTopic{"config/game_settings"};
constexpr auto kFieldDimensionsTopic{"config/field_dimensions"};

constexpr auto kGameSettingsSrv{"config/set_game_settings"};
constexpr auto kFieldDimensionsSrv{"config/set_field_dimensions"};
}  // namespace config_server::topics

namespace sim::topics {

constexpr auto kSimPlacementSrv{"sim/placement"};

}  // namespace sim::topics

namespace viz::topics {

constexpr auto kDebugDrawTopic{"viz/debug_draw"};

}  // namespace viz::topics

namespace referee::topics {
constexpr auto kPlayStateTopic{"referee/play_state"};
constexpr auto kMatchStateTopic{"referee/match_state"};
constexpr auto kOurInfoTopic{"referee/our_info"};
constexpr auto kTheirInfoTopic{"referee/their_info"};
constexpr auto kRefereeRawTopic{"referee/raw_protobuf"};
constexpr auto kGoalieTopic{"referee/our_goalie"};
constexpr auto kTeamColorTopic{"referee/team_color"};

constexpr auto kQuickCommandsSrv{"referee/quick_commands"};
}  // namespace referee::topics

namespace vision_receiver::topics {
constexpr auto kRawProtobufTopic{"vision_receiver/raw_protobuf"};
constexpr auto kDetectionFrameTopic{"vision_receiver/detection_frame"};
}  // namespace vision_receiver::topics

namespace vision_filter::topics {
constexpr auto kWorldStateTopic{"vision_filter/world_state"};
}  // namespace vision_filter::topics

namespace gameplay::topics {

static inline std::string robot_intent_topic(int robot_id) {
    return "gameplay/robot_intent/robot_" + std::to_string(robot_id);
}

constexpr auto kDebugTextTopic{"gameplay/debug_text"};

}  // namespace gameplay::topics

namespace planning::topics {

constexpr auto kGlobalObstaclesTopic{"planning/global_obstacles"};
constexpr auto kDefAreaObstaclesTopic{"planning/def_area_obstacles"};

static inline std::string trajectory_topic(int robot_id) {
    return "planning/trajectory/robot_" + std::to_string(robot_id);
}

}  // namespace planning::topics

namespace strategy::topics {
}  // namespace strategy::topics

namespace control {

namespace topics {

static inline std::string manipulator_setpoint_topic(int robot_id) {
    return "control/manipulator_setpoint/robot_" + std::to_string(robot_id);
}

static inline std::string motion_setpoint_topic(int robot_id) {
    return "control/motion_setpoint/robot_" + std::to_string(robot_id);
}

static inline std::string desired_state_topic(int robot_id) {
    return "control/desired_state/robot_" + std::to_string(robot_id);
}

static inline std::string robot_controlled_topic(int robot_id) {
    return "control/robot_controlled/robot_" + std::to_string(robot_id);
}

}  // namespace topics

namespace params {

constexpr auto kMotionControlParamModule = "motion_control";
}  // namespace params

}  // namespace control

namespace radio::topics {

static inline std::string robot_status_topic(int robot_id) {
    return "radio/robot_status/robot_" + std::to_string(robot_id);
}

}  // namespace radio::topics
