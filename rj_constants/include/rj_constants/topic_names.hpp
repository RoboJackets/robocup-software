#pragma once

#include <string>

/**
 * \file This file contains all the topic name strings to avoid typos.
 */

namespace config_server::topics {
inline constexpr auto kGameSettingsTopic{"config/game_settings"};
inline constexpr auto kFieldDimensionsTopic{"config/field_dimensions"};

inline constexpr auto kGameSettingsSrv{"config/set_game_settings"};
inline constexpr auto kFieldDimensionsSrv{"config/set_field_dimensions"};
}  // namespace config_server::topics

namespace sim::topics {

inline constexpr auto kSimPlacementSrv{"sim/placement"};

}  // namespace sim::topics

namespace viz::topics {

inline constexpr auto kDebugDrawTopic{"viz/debug_draw"};

}  // namespace viz::topics

namespace referee::topics {
inline constexpr auto kPlayStateTopic{"referee/play_state"};
inline constexpr auto kMatchStateTopic{"referee/match_state"};
inline constexpr auto kOurInfoTopic{"referee/our_info"};
inline constexpr auto kTheirInfoTopic{"referee/their_info"};
inline constexpr auto kRefereeRawTopic{"referee/raw_protobuf"};
inline constexpr auto kGoalieTopic{"referee/our_goalie"};
inline constexpr auto kTeamColorTopic{"referee/team_color"};

inline constexpr auto kQuickCommandsSrv{"referee/quick_commands"};
}  // namespace referee::topics

namespace vision_receiver::topics {
inline constexpr auto kRawProtobufTopic{"vision_receiver/raw_protobuf"};
inline constexpr auto kDetectionFrameTopic{"vision_receiver/detection_frame"};
}  // namespace vision_receiver::topics

namespace vision_filter::topics {
inline constexpr auto kWorldStateTopic{"vision_filter/world_state"};
}  // namespace vision_filter::topics

namespace gameplay::topics {

static inline std::string robot_intent_topic(int robot_id) {
    return "gameplay/robot_intent/robot_" + std::to_string(robot_id);
}

inline constexpr auto kDebugTextTopic{"gameplay/debug_text"};

}  // namespace gameplay::topics

namespace planning::topics {

inline constexpr auto kGlobalObstaclesTopic{"planning/global_obstacles"};
inline constexpr auto kDefAreaObstaclesTopic{"planning/def_area_obstacles"};

static inline std::string trajectory_topic(int robot_id) {
    return "planning/trajectory/robot_" + std::to_string(robot_id);
}

}  // namespace planning::topics

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

inline constexpr auto kMotionControlParamModule = "motion_control";
}  // namespace params

}  // namespace control

namespace radio::topics {

inline constexpr auto kAliveRobotsTopic{"radio/alive_robots"};

static inline std::string robot_status_topic(int robot_id) {
    return "radio/robot_status/robot_" + std::to_string(robot_id);
}

}  // namespace radio::topics
