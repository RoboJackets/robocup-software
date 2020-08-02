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
constexpr auto kTeamColorPub = "referee/team_color";
}  // namespace referee::topics

namespace vision_receiver::topics {
constexpr auto kRawProtobufPub = "vision_receiver/raw_protobuf";
constexpr auto kDetectionFramePub = "vision_receiver/detection_frame";
}  // namespace vision_receiver::topics

namespace vision_filter::topics {
constexpr auto kWorldStatePub = "vision_filter/world_state";
}  // namespace vision_filter::topics
