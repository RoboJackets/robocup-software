#pragma once

namespace config_server {
namespace topics {
constexpr auto kGameSettingsPub = "config/game_settings";
constexpr auto kFieldDimensionsPub = "config/field_dimensions";

constexpr auto kGameSettingsSrv = "config/set_game_settings";
constexpr auto kFieldDimensionsSrv = "config/set_field_dimensions";
}  // namespace topics
}  // namespace config_server
