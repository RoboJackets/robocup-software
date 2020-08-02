#include <config_server/config_server.h>

#include <rclcpp/rclcpp.hpp>
#include <rj_common/Field_Dimensions.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_utils/logging.hpp>

namespace config_server {
ConfigServer::ConfigServer(const rclcpp::NodeOptions& node_options)
    : Node{"config_server", node_options},
      field_dimensions_{
          rj_convert::convert_to_ros<Field_Dimensions, Field_Dimensions::Msg>(
              Field_Dimensions::Default_Dimensions)} {
    const auto latching_qos = rclcpp::QoS(1).transient_local();

    // Game Settings
    game_settings_publisher_ = create_publisher<GameSettingsMsg>(
        topics::kGameSettingsPub, latching_qos);

    // NOLINTS below are to disable performance-unnecessary-value-param
    const auto game_settings_cb =
        [this](const SetGameSettingsSrvReqPtr request,    // NOLINT
               SetGameSettingsSrvRespPtr /*response*/) {  // NOLINT
            setGameSettingsCallback(request->game_settings);
        };
    game_settings_server_ = create_service<SetGameSettingsSrv>(
        topics::kGameSettingsSrv, game_settings_cb);

    // Field Dimensions
    field_dimensions_publisher_ = create_publisher<FieldDimensionsMsg>(
        topics::kFieldDimensionsPub, latching_qos);

    // NOLINTS below are to disable performance-unnecessary-value-param
    const auto field_dimensions_cb =
        [this](const SetFieldDimensionsSrvReqPtr request,    // NOLINT
               SetFieldDimensionsSrvRespPtr /*response*/) {  // NOLINT
            setFieldDimensionsCallback(request->field_dimensions);
        };
    field_dimensions_server_ = create_service<SetFieldDimensionsSrv>(
        topics::kFieldDimensionsSrv, field_dimensions_cb);

    broadcastGameSettings();
    broadcastFieldDimensions();

    EZ_INFO("config_server is up!");
}

void ConfigServer::broadcastGameSettings() {
    game_settings_publisher_->publish(game_settings_);
}

void ConfigServer::setGameSettingsCallback(const GameSettingsMsg& msg) {
    game_settings_ = msg;
    broadcastGameSettings();
}

void ConfigServer::broadcastFieldDimensions() {
    field_dimensions_publisher_->publish(field_dimensions_);
}

void ConfigServer::setFieldDimensionsCallback(const FieldDimensionsMsg& msg) {
    field_dimensions_ = msg;
    broadcastFieldDimensions();
}

}  // namespace config_server
