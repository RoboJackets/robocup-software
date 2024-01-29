#include <rclcpp/rclcpp.hpp>

#include <config_server/config_server.hpp>
#include <rj_common/field_dimensions.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_utils/logging_macros.hpp>

namespace config_server {
ConfigServer::ConfigServer(const rclcpp::NodeOptions& node_options,
                           const GameSettingsMsg& game_settings)
    : Node{"config_server", node_options},
      game_settings_{game_settings},
      field_dimensions_{rj_convert::convert_to_ros(FieldDimensions::kDefaultDimensions)} {
    const auto latching_qos = rclcpp::QoS(1).transient_local();
    // Game Settings
    game_settings_publisher_ =
        create_publisher<GameSettingsMsg>(topics::kGameSettingsTopic, latching_qos);

    // NOLINTS below are to disable performance-unnecessary-value-param
    const auto game_settings_cb = [this](const SetGameSettingsSrvReqPtr request,    // NOLINT
                                         SetGameSettingsSrvRespPtr /*response*/) {  // NOLINT
        set_game_settings_callback(request->game_settings);
    };
    game_settings_server_ =
        create_service<SetGameSettingsSrv>(topics::kGameSettingsSrv, game_settings_cb);

    // Field Dimensions
    field_dimensions_publisher_ =
        create_publisher<FieldDimensionsMsg>(topics::kFieldDimensionsTopic, latching_qos);

    // NOLINTS below are to disable performance-unnecessary-value-param
    const auto field_dimensions_cb = [this](const SetFieldDimensionsSrvReqPtr request,    // NOLINT
                                            SetFieldDimensionsSrvRespPtr /*response*/) {  // NOLINT
        set_field_dimensions_callback(request->field_dimensions);
    };
    field_dimensions_server_ =
        create_service<SetFieldDimensionsSrv>(topics::kFieldDimensionsSrv, field_dimensions_cb);

    broadcast_game_settings();
    broadcast_field_dimensions();

    EZ_INFO("config_server is up!");
}

void ConfigServer::broadcast_game_settings() { game_settings_publisher_->publish(game_settings_); }

void ConfigServer::set_game_settings_callback(const GameSettingsMsg& msg) {
    game_settings_ = msg;
    broadcast_game_settings();
}

void ConfigServer::broadcast_field_dimensions() {
    field_dimensions_publisher_->publish(field_dimensions_);
}

void ConfigServer::set_field_dimensions_callback(const FieldDimensionsMsg& msg) {
    if (field_dimensions_ != msg) {
        field_dimensions_ = msg;
        broadcast_field_dimensions();
    }
}

}  // namespace config_server
