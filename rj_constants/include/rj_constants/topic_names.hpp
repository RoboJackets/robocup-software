#pragma once

/**
 * \file This file contains all the topic name strings to avoid typos.
 */

namespace config_server::topics {
constexpr auto kGameSettingsPub = "config/game_settings";
constexpr auto kFieldDimensionsPub = "config/field_dimensions";

constexpr auto kGameSettingsSrv = "config/set_game_settings";
constexpr auto kFieldDimensionsSrv = "config/set_field_dimensions";
}  // namespace config_server::topics

namespace referee::topics {
constexpr auto kGameStatePub = "referee/game_state";
constexpr auto kOurInfoPub = "referee/our_info";
constexpr auto kTheirInfoPub = "referee/their_info";
constexpr auto kRefereeRawPub = "referee/raw_protobuf";
constexpr auto kGoaliePub = "referee/our_goalie";
constexpr auto kTeamColorPub = "referee/team_color";

constexpr auto kQuickCommandsSrv = "referee/quick_commands";
constexpr auto kQuickRestartSrv = "referee/quick_restart";
} // namespace referee

namespace vision_receiver::topics {
constexpr auto kRawProtobufPub = "vision/raw_protobuf";
constexpr auto kDetectionFramePub = "vision/detection_frame";
}  // namespace vision_receiver::topics
