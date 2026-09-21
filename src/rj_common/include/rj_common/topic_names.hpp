#pragma once

#include <string>


namespace config_server::topics {
constexpr const char* kGameSettingsTopic{"/config/game_settings"};
constexpr const char* kFieldDimensionsTopic{"/config/field_dimensions"};
constexpr const char* kGameSettingsSrv{"/config/set_game_settings"};
constexpr const char* kFieldDimensionsSrv{"/config/set_field_dimensions"};
}  // namespace config_server::topics

namespace sim::topics {
constexpr const char* kSimPlacementSrv{"/sim/placement"};
}  // namespace sim::topics

namespace viz::topics {
constexpr const char* kDebugDrawTopic{"/viz/debug_draw"};
}  // namespace viz::topics

namespace referee::topics {
constexpr const char* kPlayStateTopic{"/referee/play_state"};
constexpr const char* kMatchStateTopic{"/referee/match_state"};
constexpr const char* kOurInfoTopic{"/referee/our_info"};
constexpr const char* kTheirInfoTopic{"/referee/their_info"};
constexpr const char* kRefereeRawTopic{"/referee/raw_protobuf"};
constexpr const char* kGoalieTopic{"/referee/our_goalie"};
constexpr const char* kTeamColorTopic{"/referee/team_color"};
constexpr const char* kQuickCommandsSrv{"/referee/quick_commands"};
}  // namespace referee::topics

namespace vision_receiver::topics {
constexpr const char* kRawProtobufTopic{"/vision_receiver/raw_protobuf"};
constexpr const char* kDetectionFrameTopic{"/vision_receiver/detection_frame"};
}  // namespace vision_receiver::topics

namespace vision_filter::topics {
constexpr const char* kWorldStateTopic{"/vision_filter/world_state"};
}  // namespace vision_filter::topics

namespace gameplay::topics {
static inline std::string robot_intent_topic(int robot_id) {
    return "/gameplay/robot_intent/robot_" + std::to_string(robot_id);
}
constexpr const char* kDebugTextTopic{"/gameplay/debug_text"};
}  // namespace gameplay::topics

namespace planning::topics {
static inline std::string trajectory_topic(int robot_id) {
    return "/planning/trajectory/robot_" + std::to_string(robot_id);
}
constexpr const char* kDefAreaObstaclesTopic{"/planning/def_area_obstacles"};
}  // namespace planning::topics

namespace control::topics {
static inline std::string manipulator_setpoint_topic(int robot_id) {
    return "/control/manipulator_setpoint/robot_" + std::to_string(robot_id);
}
static inline std::string motion_setpoint_topic(int robot_id) {
    return "/control/motion_setpoint/robot_" + std::to_string(robot_id);
}
static inline std::string desired_state_topic(int robot_id) {
    return "/control/desired_state/robot_" + std::to_string(robot_id);
}
static inline std::string robot_controlled_topic(int robot_id) {
    return "/control/robot_controlled/robot_" + std::to_string(robot_id);
}
}  // namespace control::topics

namespace control::params {
constexpr const char* kMotionControlParamModule{"/motion_control"};
}  // namespace control::params

namespace radio::topics {
static inline std::string robot_status_topic(int robot_id) {
    return "/radio/robot_status/robot_" + std::to_string(robot_id);
}
constexpr const char* kAliveRobotsTopic{"/radio/alive_robots"};
}  // namespace radio::topics
